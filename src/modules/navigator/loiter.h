/***************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file loiter.h
 *
 * Helper class to loiter
 *
 * @author Julian Oes <julian@oes.ch>
 */

#ifndef NAVIGATOR_LOITER_H
#define NAVIGATOR_LOITER_H

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#include "navigator_mode.h"
#include "mission_block.h"

class Loiter : public MissionBlock
{
public:
	Loiter(Navigator *navigator, const char *name);

	~Loiter();

	virtual void on_inactive();

	virtual void on_activation();

	virtual void on_active();

	enum mission_yaw_mode {
		MISSION_YAWMODE_NONE = 0,
		MISSION_YAWMODE_FRONT_TO_WAYPOINT = 1,
		MISSION_YAWMODE_FRONT_TO_HOME = 2,
		MISSION_YAWMODE_BACK_TO_HOME = 3,
		MISSION_YAWMODE_MAX = 4
	};

private:
	/**
	 * Use the stored reposition location of the navigator
	 * to move to a new location.
	 */
	void reposition();

	/**
	 * Set the position to hold based on the current local position
	 */
	void set_loiter_position();

	control::BlockParamFloat _param_min_alt;
	control::BlockParamInt _param_yawmode;
	bool _loiter_pos_set;
};

#endif
