#include "active_3d_planning/module/trajectory_evaluator/evaluator_updater/yaw_planning_updaters.h"

#include "active_3d_planning/planner/planner_I.h"
#include "active_3d_planning/module/module_factory.h"

namespace active_3d_planning {
namespace evaluator_updater {
using YawPlanningInfo =
    active_3d_planning::trajectory_evaluator::YawPlanningInfo;

// YawPlanningUpdateAdapter
ModuleFactoryRegistry::Registration<YawPlanningUpdateAdapter>
    YawPlanningUpdateAdapter::registration("YawPlanningUpdateAdapter");

YawPlanningUpdateAdapter::YawPlanningUpdateAdapter(PlannerI &planner)
    : EvaluatorUpdater(planner) {}

bool YawPlanningUpdateAdapter::updateSegments(TrajectorySegment *root) {
  // recursively update all segments, exclude root since this is now the base
  // with no parent
  for (int i = 0; i < root->children.size(); ++i) {
    updateSingle(root->children[i].get());
  }
  return true;
}

void YawPlanningUpdateAdapter::setupFromParamMap(Module::ParamMap *param_map) {
  setParam<bool>(param_map, "dynamic_trajectories", &p_dynamic_trajectories_,
                 false);
  // Create Following updater (default does nothing)
  std::string args; // default args extends the parent namespace
  std::string param_ns = (*param_map)["param_namespace"];
  setParam<std::string>(param_map, "following_updater_args", &args,
                        param_ns + "/following_updater");
  following_updater_ = planner_.getFactory().createModule<EvaluatorUpdater>(
      args, planner_, verbose_modules_);
}

void YawPlanningUpdateAdapter::updateSingle(TrajectorySegment *segment) {
  if (segment->info) {
    YawPlanningInfo *info =
        reinterpret_cast<YawPlanningInfo *>(segment->info.get());
    if (p_dynamic_trajectories_) {
      // Update the (possibly changed) trajectory
      info->orientations[info->active_orientation].trajectory =
          segment->trajectory;
      info->orientations[info->active_orientation].parent = segment->parent;
    }
    // Update the stored active orientation
    info->orientations[info->active_orientation].parent = segment->parent;
    following_updater_->updateSegments(
        &(info->orientations[info->active_orientation]));
    segment->gain = info->orientations[info->active_orientation].gain;
    segment->cost = info->orientations[info->active_orientation].cost;
    segment->value = info->orientations[info->active_orientation].value;
  }

  // Propagate through trajectory tree
  for (int i = 0; i < segment->children.size(); ++i) {
    updateSingle(segment->children[i].get());
  }
}

// YawPlanningUpdater
ModuleFactoryRegistry::Registration<YawPlanningUpdater>
    YawPlanningUpdater::registration("YawPlanningUpdater");

YawPlanningUpdater::YawPlanningUpdater(PlannerI &planner)
    : EvaluatorUpdater(planner) {}

bool YawPlanningUpdater::updateSegments(TrajectorySegment *root) {
  // recursively update all segments, exclude root since this is now the base
  // with no parent
  updateSingle(root);
  return following_updater_->updateSegments(root);
}

void YawPlanningUpdater::setupFromParamMap(Module::ParamMap *param_map) {
  setParam<bool>(param_map, "select_by_value", &p_select_by_value_, false);
  setParam<bool>(param_map, "dynamic_trajectories", &p_dynamic_trajectories_,
                 false);
  setParam<double>(param_map, "update_range", &p_update_range_,
                   -1.0); // default does not update
  setParam<double>(param_map, "update_gain", &p_update_gain_, 0.0);

  // Create Following updater (default does nothing)
  std::string args; // default args extends the parent namespace
  std::string param_ns = (*param_map)["param_namespace"];
  setParam<std::string>(param_map, "following_updater_args", &args,
                        param_ns + "/following_updater");
  following_updater_ = planner_.getFactory().createModule<EvaluatorUpdater>(
      args, planner_, verbose_modules_);
}

void YawPlanningUpdater::updateSingle(TrajectorySegment *segment) {
  if (segment->info && segment->parent != nullptr) {
    YawPlanningInfo *info =
        reinterpret_cast<YawPlanningInfo *>(segment->info.get());

    // If requested, check whether the trajectory changed
    if (p_dynamic_trajectories_) {
      if (info->orientations[info->active_orientation].gain != segment->gain ||
          info->orientations[info->active_orientation].parent !=
              segment->parent) {
        // Trajectory changed, recompute all orientations
        for (int j = 0; j < info->orientations.size(); ++j) {
          info->orientations[j].parent = segment->parent;
          double yaw = info->orientations[j].trajectory.back().getYaw();
          info->orientations[j].trajectory = segment->trajectory;
          static_cast<YawPlanningEvaluator&>(
              planner_.getTrajectoryEvaluator())
              .setTrajectoryYaw(&(info->orientations[j]),
                                segment->trajectory.front().getYaw(), yaw);
        }
      }
    }

    // Update all orientations with the view updater and select the best one as
    // the main trajectory Initialization step
    bool in_reach =
        (planner_.getCurrentPosition() - segment->trajectory.back().position_W)
            .norm() <= p_update_range_;
    if (in_reach && info->orientations[0].gain > p_update_gain_) {
      static_cast<YawPlanningEvaluator&>(planner_.getTrajectoryEvaluator())
          .following_evaluator_->computeGain(&(info->orientations[0]));
    }
    info->active_orientation = 0;
    double best_value = info->orientations[0].gain;
    if (p_select_by_value_) {
      best_value = info->orientations[0].value;
    }

    // Update and compare all other orientations
    for (int i = 1; i < info->orientations.size(); ++i) {
      if (in_reach && info->orientations[i].gain > p_update_gain_) {
        static_cast<YawPlanningEvaluator&>(planner_.getTrajectoryEvaluator())
            .following_evaluator_->computeGain(&(info->orientations[i]));
      }
      if (p_select_by_value_) {
        static_cast<YawPlanningEvaluator&>(planner_.getTrajectoryEvaluator())
            .following_evaluator_->computeCost(&(info->orientations[i]));
        static_cast<YawPlanningEvaluator&>(planner_.getTrajectoryEvaluator())
            .following_evaluator_->computeValue(&(info->orientations[i]));
        if (info->orientations[i].value > best_value) {
          info->active_orientation = i;
          best_value = info->orientations[i].value;
        }
      } else {
        if (info->orientations[i].gain > best_value) {
          info->active_orientation = i;
          best_value = info->orientations[i].gain;
        }
      }
    }

    // Apply best rotation to the segment
    segment->trajectory =
        info->orientations[info->active_orientation].trajectory;
    segment->gain = info->orientations[info->active_orientation].gain;
    if (p_select_by_value_) {
      segment->cost = info->orientations[info->active_orientation].cost;
      segment->value = info->orientations[info->active_orientation].value;
    }
  }

  // Propagate through trajectory tree
  for (int i = 0; i < segment->children.size(); ++i) {
    updateSingle(segment->children[i].get());
  }
}

} // namespace evaluator_updater
} // namespace active_3d_planning