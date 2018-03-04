/*
 *  Copyright 2017, Magazino GmbH, Sebastian Pütz, Jorge Santos Simón
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  abstract_recovery_execution.tcc
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include <boost/exception/diagnostic_information.hpp>

#include "mbf_abstract_nav/abstract_recovery_execution.h"

namespace mbf_abstract_nav
{


  AbstractRecoveryExecution::AbstractRecoveryExecution(
      boost::condition_variable &condition,
      const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr) :
      tf_listener_ptr_(tf_listener_ptr), state_(STOPPED), canceled_(false),
      AbstractPluginHandler<mbf_abstract_core::AbstractRecovery>(condition)
  {
  }


  AbstractRecoveryExecution::~AbstractRecoveryExecution()
  {
  }

  bool AbstractRecoveryExecution::initialize()
  {
    if(loadPlugins("recovery_behaviors"))
    {
      setState(INITIALIZED);
      return true;
    }
    return false;
  }


  void AbstractRecoveryExecution::reconfigure(const MoveBaseFlexConfig &config)
  {
    boost::recursive_mutex::scoped_lock sl(configuration_mutex_);

    // Nothing to do here, as recovery_enabled is loaded and used in the navigation server
  }


  void AbstractRecoveryExecution::setState(RecoveryState state)
  {
    boost::lock_guard<boost::mutex> guard(state_mtx_);
    state_ = state;
  }


  typename AbstractRecoveryExecution::RecoveryState AbstractRecoveryExecution::getState()
  {
    boost::lock_guard<boost::mutex> guard(state_mtx_);
    return state_;
  }


  void AbstractRecoveryExecution::startRecovery(const std::string &name)
  {
    requested_behavior_name_ = name;
    setState(STARTED);
    plugin_thread_ = boost::thread(&AbstractRecoveryExecution::run, this);
  }


  void AbstractRecoveryExecution::stopRecovery()
  {
    plugin_thread_.interrupt();
    setState(STOPPED);
  }


  bool AbstractRecoveryExecution::cancel()
  {
    canceled_ = true;
    if (plugin_.second)
    {
      // returns false if cancel is not implemented or rejected by the recovery behavior (will run until completion)
      return plugin_.second->cancel();
    }
    return false;
  }


  bool AbstractRecoveryExecution::hasRecoveryBehavior(const std::string &name)
  {
    return plugins_.find(name) != plugins_.end();
  }


  std::vector<std::string> AbstractRecoveryExecution::listRecoveryBehaviors()
  {
    std::vector<std::string> recovery_behaviors;
    for (NameTypeMap::iterator it = plugin_types_.begin(); it != plugin_types_.end(); ++it)
    {
      recovery_behaviors.push_back(it->first);
    }
    return recovery_behaviors;
  }


  bool AbstractRecoveryExecution::getTypeOfBehavior(const std::string &name, std::string &type)
  {
    NameTypeMap::iterator finder = plugin_types_.find(name);
    if (finder != plugin_types_.end())
    {
      type = finder->second;
      return true;
    }
    return false;
  }


  void AbstractRecoveryExecution::run()
  {
    boost::recursive_mutex::scoped_lock sl(configuration_mutex_);
    canceled_ = false; // (re)set the canceled state

    NamePluginMap::iterator find_iter = plugins_.find(requested_behavior_name_);

    if (find_iter == plugins_.end())
    {
      // no such recovery behavior
      ROS_ERROR_STREAM("No recovery behavior for the given name: \"" << requested_behavior_name_ << "\"!");
      setState(WRONG_NAME);
      condition_.notify_one();
      return;
    }

    plugin_ = *find_iter;
    setState(RECOVERING);
    try
    {
      // TODO use outcome and message
      std::string message;
      uint32_t outcome = plugin_.second->runBehavior(message);
      if (canceled_)
      {
        setState(CANCELED);
      }
      else
      {
        setState(RECOVERY_DONE);
      }
    }
    catch (boost::thread_interrupted &ex)
    {
      setState(STOPPED);
    }
    catch (...){
      ROS_FATAL_STREAM("Unknown error occurred: " << boost::current_exception_diagnostic_information());
      setState(INTERNAL_ERROR);
    }
    condition_.notify_one();
    plugin_.second.reset();
  }
} /* namespace mbf_abstract_nav */
