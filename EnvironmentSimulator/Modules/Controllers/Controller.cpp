/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#include "Controller.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"
#include "ScenarioEngine.hpp"
#include "logger.hpp"

#include "ControllerExternal.hpp"
#include "ControllerFollowGhost.hpp"
#include "ControllerFollowRoute.hpp"
#include "ControllerInteractive.hpp"
#include "ControllerSloppyDriver.hpp"
#include "ControllerRel2Abs.hpp"
#include "ControllerACC.hpp"
#include "ControllerNaturalDriver.hpp"
#include "ControllerALKS.hpp"
#include "ControllerUDPDriver.hpp"
#include "ControllerALKS_R157SM.hpp"
#include "ControllerLooming.hpp"
#include "ControllerOffroadFollower.hpp"
#include "ControllerHID.hpp"
#include "ControllerFollowReference.hpp"

using namespace scenarioengine;

Controller* Controller::Create(const std::string& type, const std::string& name)
{
    Controller::InitArgs args;
    args.name = name;
    args.type = type;
    args.properties = nullptr;
    args.gateway = nullptr;
    args.scenario_engine = nullptr;
    args.parameters = nullptr;

    if (type == "External")           return new ControllerExternal(&args);
    if (type == "FollowGhost")        return new ControllerFollowGhost(&args);
    if (type == "FollowRoute")        return new ControllerFollowRoute(&args);
    if (type == "Interactive")        return new ControllerInteractive(&args);
    if (type == "SloppyDriver")       return new ControllerSloppyDriver(&args);
    if (type == "Rel2Abs")            return new ControllerRel2Abs(&args);
    if (type == "ACC")                return new ControllerACC(&args);
    if (type == "NaturalDriver")      return new ControllerNaturalDriver(&args);
    if (type == "ALKS")               return new ControllerALKS(&args);
    if (type == "UDPDriver")          return new ControllerUDPDriver(&args);
    if (type == "ALKS_R157SM")         return new ControllerALKS_R157SM(&args);
    if (type == "Looming")            return new ControllerLooming(&args);
    if (type == "OffroadFollower")    return new ControllerOffroadFollower(&args);
    if (type == "HID")                return new ControllerHID(&args);
    if (type == "FollowReference")    return new ControllerFollowReference(&args);
    
    LOG_WARN("Unknown controller type: {}", type);
    return nullptr;
}

Controller* scenarioengine::InstantiateController(void* args)
{
    LOG_ERROR("The base class should not be instantiated");

    return new Controller(static_cast<Controller::InitArgs*>(args));
}

Controller::Controller(InitArgs* args)  // init operatingdomains
    : operating_domains_(static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LAT_AND_LONG)),
      active_domains_(static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_NONE)),
      mode_(ControlOperationMode::MODE_OVERRIDE),
      object_(nullptr),
      entities_(nullptr),
      gateway_(nullptr),
      scenario_engine_(nullptr),
      player_(nullptr)
{
    if (args)
    {
        name_            = args->name;
        type_name_       = args->type;
        gateway_         = args->gateway;
        scenario_engine_ = args->scenario_engine;
        entities_        = scenario_engine_ != nullptr ? &scenario_engine_->entities_ : nullptr;
    }
    else
    {
        LOG_ERROR_AND_QUIT("Controller constructor missing args");
    }

    if (args->properties && args->properties->ValueExists("mode"))
    {
        std::string mode = args->properties->GetValueStr("mode");
        if (mode == "override")
        {
            mode_ = ControlOperationMode::MODE_OVERRIDE;
        }
        else if (mode == "additive")
        {
            mode_ = ControlOperationMode::MODE_ADDITIVE;
        }
        else
        {
            LOG_WARN("Unexpected mode \"{}\", falling back to default \"override\"", mode);
            mode_ = ControlOperationMode::MODE_OVERRIDE;
        }
    }
    else
    {
        mode_ = ControlOperationMode::MODE_OVERRIDE;
    }
}

void Controller::Step(double timeStep)
{
    (void)timeStep;
    if (object_)
    {
        if (mode_ == ControlOperationMode::MODE_OVERRIDE)
        {
            if (IsActiveOnDomains(static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LAT)))
            {
                object_->SetDirtyBits(Object::DirtyBit::LATERAL);
            }

            if (IsActiveOnDomains(static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LONG)))
            {
                object_->SetDirtyBits(Object::DirtyBit::LONGITUDINAL);
            }
        }
        else
        {
            object_->ClearDirtyBits(Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL);
        }
    }
}

void Controller::LinkObject(Object* object)
{
    object_ = object;
}

void Controller::UnlinkObject()
{
    object_ = nullptr;
}

int Controller::Activate(const ControlActivationMode (&mode)[static_cast<unsigned int>(ControlDomains::COUNT)])
{
    if (mode[static_cast<unsigned int>(ControlDomains::DOMAIN_LAT)] == ControlActivationMode::OFF && align_to_road_heading_on_deactivation_)
    {
        // Make sure heading is aligned with driving direction when controller is deactivated on the lateral domain
        if (IsActiveOnDomains(static_cast<int>(ControlDomainMasks::DOMAIN_MASK_LAT)))
        {
            AlignToRoadHeading();
        }
    }

    if (mode[static_cast<unsigned int>(ControlDomains::DOMAIN_LAT)] == ControlActivationMode::ON && align_to_road_heading_on_activation_)
    {
        // Make sure heading is aligned with driving direction when controller is activated on the lateral domain
        AlignToRoadHeading();
    }

    for (unsigned int i = 0; i < static_cast<unsigned int>(ControlDomains::COUNT); i++)
    {
        if (mode[i] == ControlActivationMode::OFF)
        {
            active_domains_ &= ~(static_cast<unsigned int>(ControlDomain2DomainMask(static_cast<ControlDomains>(i))));
        }
        else if (mode[i] == ControlActivationMode::ON)
        {
            if ((operating_domains_ & static_cast<unsigned int>(ControlDomain2DomainMask(static_cast<ControlDomains>(i)))) == 0)
            {
                LOG_WARN("Warning: Controller {} operating domains: {}. Skipping activation on domain {}",
                         GetName(),
                         ControlDomainMask2Str(operating_domains_),
                         ControlDomain2Str(static_cast<ControlDomains>(i)));
            }
            else
            {
                active_domains_ |= static_cast<unsigned int>(ControlDomain2DomainMask(static_cast<ControlDomains>(i)));
            }
        }
    }
    return 0;
}

void scenarioengine::Controller::DeactivateDomains(unsigned int domains)
{
    // Make sure heading is aligned with driving direction when controller is deactivated on the lateral domain
    if (align_to_road_heading_on_deactivation_ && IsActiveOnDomains(static_cast<int>(ControlDomainMasks::DOMAIN_MASK_LAT)) &&
        (domains & static_cast<int>(ControlDomainMasks::DOMAIN_MASK_LAT)))
    {
        AlignToRoadHeading();
    }

    active_domains_ = active_domains_ & ~domains;
}

void Controller::ReportKeyEvent(int key, bool down)
{
    LOG_DEBUG("Key {} {}", key, down ? "down" : "up");
}

std::string Controller::Mode2Str(ControlOperationMode mode)
{
    if (mode == ControlOperationMode::MODE_OVERRIDE)
    {
        return "override";
    }
    else if (mode == ControlOperationMode::MODE_ADDITIVE)
    {
        return "additive";
    }
    else if (mode == ControlOperationMode::MODE_NONE)
    {
        return "none";
    }
    else
    {
        LOG_ERROR("Unexpected mode \"{}\"", std::to_string(static_cast<int>(mode)));
        return "invalid mode";
    }
}

bool Controller::IsActiveOnDomainsOnly(unsigned int domainMask) const
{
    return (GetActiveDomains() == domainMask);
}

bool Controller::IsActiveOnDomains(unsigned int domainMask) const
{
    return (domainMask & GetActiveDomains()) == domainMask;
}

bool Controller::IsNotActiveOnDomains(unsigned int domainMask) const
{
    return (domainMask & GetActiveDomains()) == 0;
}

bool Controller::IsActiveOnAnyOfDomains(unsigned int domainMask) const
{
    return (domainMask & GetActiveDomains()) != 0;
}

bool Controller::IsActive() const
{
    return GetActiveDomains() != static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_NONE);
}

void scenarioengine::Controller::AlignToRoadHeading()
{
    if (object_ != nullptr)
    {
        object_->pos_.SetHeading(object_->pos_.GetHRoadInDrivingDirection());
    }
}
