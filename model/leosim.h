/*
 * Copyright (c) 2024 Anupa De Silva
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/**
 * \defgroup leosim LEO Satellite Simulation Module
 * \ingroup contrib
 *
 * \section leosim-overview Overview
 *
 * This module provides a comprehensive simulation framework for LEO (Low Earth Orbit)
 * satellite constellations. It includes models for:
 * - Satellite mobility and orbital mechanics
 * - Inter-Satellite Links (ISLs)
 * - Ground station links
 * - Link quality and channel modeling
 * - Routing calculation and path computation
 * - Data loading and visualization
 */

#ifndef LEOSIM_H
#define LEOSIM_H

#include "leosim-beam-hopping-manager.h"
#include "leosim-beam-layout-engine.h"
#include "leosim-beam-load-balancer.h"
#include "leosim-channel.h"
#include "leosim-channel-model.h"
#include "leosim-isl-routing-model.h"
#include "leosim-loader.h"
#include "leosim-mobility-model.h"
#include "leosim-multi-beam-model.h"
#include "leosim-routing-calculator.h"
#include "leosim-sinr-engine.h"
#include "leosim-operator-model.h"

#include "../helper/leosim-operator-helper.h"

#endif /* LEOSIM_H */
