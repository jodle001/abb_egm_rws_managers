/***********************************************************************************************************************
 *
 * Copyright (c) 2020, ABB Schweiz AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that
 * the following conditions are met:
 *
 *    * Redistributions of source code must retain the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer.
 *    * Redistributions in binary form must reproduce the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer in the documentation
 *      and/or other materials provided with the
 *      distribution.
 *    * Neither the name of ABB nor the names of its
 *      contributors may be used to endorse or promote
 *      products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***********************************************************************************************************************
 */

#ifndef ABB_EGM_RWS_MANAGERS_RWS_MANAGER_H
#define ABB_EGM_RWS_MANAGERS_RWS_MANAGER_H

#include <memory>
#include <mutex>
#include <functional>
#include <string>
#include <type_traits>
#include <utility>

#include <abb_librws/v1_0/rws_state_machine_interface.h>
#include <abb_librws/v2_0/rws_state_machine_interface.h>

#include "utilities.h"

namespace abb
{
namespace robot
{
/**
 * \brief Version of the Robot Web Services protocol that a robot controller speaks.
 *
 * IRC5 controllers (RobotWare 6) serve RWS 1.0 over plain HTTP. OmniCore controllers
 * (RobotWare 7 and later) serve RWS 2.0 over TLS only.
 */
enum class RWSVersion
{
  v1_0,
  v2_0
};

/**
 * \brief Maps a station's controller generation to the RWS version it serves.
 *
 * The comparison ignores case and surrounding whitespace, so "OmniCore" selects RWS 2.0
 * the same as "omnicore" does.
 *
 * \param controller_generation as configured for the station (e.g. "irc5" or "omnicore").
 *
 * \return RWSVersion to use. Anything other than "omnicore" maps to v1.0, so stations
 *         that do not declare a generation keep their existing IRC5 behaviour.
 */
RWSVersion rwsVersionFromControllerGeneration(const std::string& controller_generation);

/**
 * \brief Whether a configured controller generation is one this library recognises.
 *
 * An unrecognised value is not an error here - it maps to RWS 1.0 like any non-OmniCore
 * controller - but it is almost always a misconfiguration, and one that shows up only as
 * a connect retry loop. Callers with somewhere to log should use this to say so.
 *
 * \param controller_generation as configured for the station.
 *
 * \return bool true if the value names a known controller generation.
 */
bool isKnownControllerGeneration(const std::string& controller_generation);

/**
 * \brief A unit of work to run against a robot controller's RWS interface.
 *
 * The two librws state machine interfaces are identical apart from their namespace, so a
 * service body written against one compiles unchanged against the other. This type erases
 * which of the two it will be handed, which keeps callers free of the version choice.
 */
class RWSService
{
public:
  virtual ~RWSService() = default;

  virtual void operator()(rws::v1_0::RWSStateMachineInterface& interface) const = 0;
  virtual void operator()(rws::v2_0::RWSStateMachineInterface& interface) const = 0;
};

/**
 * \brief Wraps a generic callable (e.g. a lambda taking 'auto&') as an RWSService.
 */
template <typename Callable>
class RWSServiceOf final : public RWSService
{
public:
  explicit RWSServiceOf(Callable callable) : callable_{ std::move(callable) }
  {
  }

  void operator()(rws::v1_0::RWSStateMachineInterface& interface) const override
  {
    callable_(interface);
  }

  void operator()(rws::v2_0::RWSStateMachineInterface& interface) const override
  {
    callable_(interface);
  }

private:
  mutable Callable callable_;
};

/**
 * \brief Manager for handling Robot Web Service (RWS) communication with an ABB robot controller.
 *
 * Version-erased interface. Callers hold one of these and never name an RWS version; the
 * concrete manager built by makeRWSManager() decides which librws interface is used.
 */
class RWSManagerBase
{
public:
  virtual ~RWSManagerBase() = default;

  /**
   * \brief Collects key data, about the robot controller's active system, and parses it into a structured description.
   *
   * \param prefix for standardized joint names (i.e. arbitrary prefix for identifying a specific robot controller).
   *
   * \return RobotControllerDescription of the robot controller.
   *
   * \throw std::runtime_error if a handleable error happened (e.g. communication timed out).
   */
  virtual RobotControllerDescription collectAndParseSystemData(const std::string& prefix) = 0;

  /**
   * \brief Collects runtime data, about the robot controller's active system, and updates the runtime data containers.
   *
   * This includes general system states and motion information of the system's mechanical units.
   *
   * \param system_state_data container for storing the general system states.
   * \param motion_data container for storing the motion states.
   *
   * \throw std::runtime_error if a handleable error happened (e.g. communication timed out).
   * \throw std::logic_error if an unhandleable error happened (e.g. switching to another robot controller system).
   */
  virtual void collectAndUpdateRuntimeData(SystemStateData& system_state_data, MotionData& motion_data) = 0;

  /**
   * \brief Checks if the low priority RWS interface is ready to be used.
   *
   * \return bool true if the interface is ready.
   */
  virtual bool isInterfaceReady() = 0;

  /**
   * \brief Runs the provided service with the low priority RWS interface.
   *
   * Notes:
   * - Only runs the service if the interface is free.
   * - Intended for lower priority services.
   *
   * \param service to run.
   *
   * \return bool true if the service was run.
   */
  template <typename Callable>
  bool runService(Callable&& service)
  {
    RWSServiceOf<typename std::decay<Callable>::type> wrapped{ std::forward<Callable>(service) };
    return runServiceImpl(wrapped);
  }

  /**
   * \brief Runs the provided service with the high priority RWS interface.
   *
   * Notes:
   * - Waits until the interface is free before running the service.
   * - Intended for higher priority services.
   *
   * \param service to run.
   */
  template <typename Callable>
  void runPriorityService(Callable&& service)
  {
    RWSServiceOf<typename std::decay<Callable>::type> wrapped{ std::forward<Callable>(service) };
    runPriorityServiceImpl(wrapped);
  }

  /**
   * \brief Creates a debug text of the latest connection attempt.
   *
   * I.e. a summary of collected key data, about the robot controller's active system,
   * and the structured description parsed from the key data.
   *
   * \return std::string containing the debug text.
   */
  virtual std::string debugText() const = 0;

protected:
  virtual bool runServiceImpl(RWSService const& service) = 0;
  virtual void runPriorityServiceImpl(RWSService const& service) = 0;
};

/**
 * \brief Concrete RWS manager, bound to one librws interface version.
 *
 * \tparam Interface librws state machine interface type (v1_0 or v2_0).
 * \tparam Client librws client type matching Interface.
 */
template <typename Interface, typename Client>
class RWSManagerT : public RWSManagerBase
{
public:
  /**
   * \brief Creates a manager for handling communication with the robot controller's RWS server.
   *
   * \param ip_address to the RWS server.
   * \param port_number used by the RWS server.
   * \param username for the RWS authentication process.
   * \param password for the RWS authentication process.
   */
  RWSManagerT(const std::string& ip_address, const unsigned short port_number, const std::string& username,
              const std::string& password);

  RobotControllerDescription collectAndParseSystemData(const std::string& prefix) override;

  void collectAndUpdateRuntimeData(SystemStateData& system_state_data, MotionData& motion_data) override;

  bool isInterfaceReady() override;

  std::string debugText() const override;

protected:
  bool runServiceImpl(RWSService const& service) override;

  void runPriorityServiceImpl(RWSService const& service) override;

private:
  /**
   * \brief Mutex for protecting the low priority RWS interface.
   */
  std::mutex interface_mutex_;

  /**
   * \brief Mutex for protecting the high priority RWS interface.
   */
  std::mutex priority_interface_mutex_;

  /**
   * \brief RWSClient intended for lower priority requests
   **/
  Client client_;

  /**
   * \brief RWSClient intended for higher priority requests
   **/
  Client priority_client_;

  /**
   * \brief RWS communication interface, intended for lower priority requests.
   */
  Interface interface_;

  /**
   * \brief RWS communication interface, intended for higher priority requests.
   */
  Interface priority_interface_;

  /**
   * \brief Key data about the robot controller's active system (in raw, unstructured, format).
   */
  SystemData system_data_;

  /**
   * \brief Structured description of the connected robot controller.
   */
  RobotControllerDescription description_;

  /**
   * \brief The robot controller's runtime system state.
   */
  SystemStateData system_state_data_;
};

using RWSManagerV1 = RWSManagerT<rws::v1_0::RWSStateMachineInterface, rws::v1_0::RWSClient>;
using RWSManagerV2 = RWSManagerT<rws::v2_0::RWSStateMachineInterface, rws::v2_0::RWSClient>;

/**
 * \brief Creates the RWS manager for a given protocol version.
 *
 * \param version of the RWS protocol the controller serves.
 * \param ip_address to the RWS server.
 * \param port_number used by the RWS server.
 * \param username for the RWS authentication process.
 * \param password for the RWS authentication process.
 *
 * \return std::unique_ptr to the created manager.
 */
std::unique_ptr<RWSManagerBase> makeRWSManager(RWSVersion version, const std::string& ip_address,
                                               const unsigned short port_number, const std::string& username,
                                               const std::string& password);

}  // namespace robot
}  // namespace abb

#endif
