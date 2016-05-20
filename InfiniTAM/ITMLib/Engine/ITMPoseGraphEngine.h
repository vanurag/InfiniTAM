/*
 * ITMPoseGraphEngine.h
 *
 *  Created on: May 17, 2016
 *      Author: anurag
 */

#ifndef INFINITAM_INFINITAM_ITMLIB_ENGINE_ITMPOSEGRAPHENGINE_H_
#define INFINITAM_INFINITAM_ITMLIB_ENGINE_ITMPOSEGRAPHENGINE_H_

#include "../Utils/ITMLibDefines.h"
#include "../Utils/ITMLibSettings.h"

#include "../Engine/ITMLowLevelEngine.h"

#include "../../Utils/gnuplot-iostream/gnuplot-iostream.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

namespace ITMLib
{
  namespace Engine
  {
    template<class T>
    class ITMPoseGraphEngine
    {
     private:
      // Create an empty nonlinear factor graph
      gtsam::NonlinearFactorGraph graph_;
      gtsam::Key current_node_;
      // Create the data structure to hold the initialEstimate estimate to the solution
      gtsam::Values initial_estimates_;
     public:
      void addPrior(const T &priorMean, const gtsam::noiseModel::Base::shared_ptr &priorNoise) {
        graph_.add(gtsam::PriorFactor<T>(1, priorMean, priorNoise));
      }
      void addOdometry(const T &odometry, const gtsam::noiseModel::Base::shared_ptr &odometryNoise) {
        graph_.add(gtsam::BetweenFactor<T>(current_node_, current_node_+1, odometry, odometryNoise));
      }
      void addOdometry(const T &odometry, const gtsam::noiseModel::Base::shared_ptr &odometryNoise, const int node_id) {
        graph_.add(gtsam::BetweenFactor<T>(node_id, node_id+1, odometry, odometryNoise));
      }
      void addInitialEstimate(const T &estimate) {
        initial_estimates_.insert(current_node_, estimate);
      }
      void addInitialEstimate(const T &estimate, const int node_id) {
        initial_estimates_.insert(node_id, estimate);
      }
      void propogate() {
        current_node_++;
      }
      void printGraph() {
        graph_.print("\nCurrent Factor Graph:\n"); // print
      }
      gtsam::Values performOptimization() {
        gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph_, initial_estimates_).optimize();
        result.print("Final Result:\n");
        return result;
      }
      ITMPoseGraphEngine() {
        current_node_ = 1;
      }
      ~ITMPoseGraphEngine();
    };
  }
}

#endif /* INFINITAM_INFINITAM_ITMLIB_ENGINE_ITMPOSEGRAPHENGINE_H_ */
