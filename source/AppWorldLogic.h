/* Copyright (C) 2005-2022, UNIGINE. All rights reserved.
 *
 * This file is a part of the UNIGINE 2 SDK.
 *
 * Your use and / or redistribution of this software in source and / or
 * binary form, with or without modification, is subject to: (i) your
 * ongoing acceptance of and compliance with the terms and conditions of
 * the UNIGINE License Agreement; and (ii) your inclusion of this notice
 * in any version of this software that you use or redistribute.
 * A copy of the UNIGINE License Agreement is available by contacting
 * UNIGINE. at http://unigine.com/
 */


#ifndef __APP_WORLD_LOGIC_H__
#define __APP_WORLD_LOGIC_H__

#include <UnigineLogic.h>
#include <UnigineStreams.h>
#include <UnigineNode.h>
#include <UnigineWidgets.h>

class AppWorldLogic: public Unigine::WorldLogic
{
public:
	const int quality = 10; // curve subdivisions

	int init() override;
	int update() override;
	int shutdown() override;

private:
	void draw_path() const;
	void update_time(float speed = 1.0f);
	Unigine::VectorStack<Unigine::Math::Vec3, 4> get_current_points() const;
	Unigine::VectorStack<Unigine::Math::quat, 4> get_current_quats() const;
	void update_position_only();
	void update_position_frenet();
	void update_position_rmf();
	void update_position_rotation();
	void update_position_squad();
	void update_final();

	// control points
	Unigine::Vector<Unigine::Math::Vec3> points_pos;
	Unigine::Vector<Unigine::Math::quat> points_rot;
	Unigine::Vector<float> lengths;

	// current position
	int points_index = 0;
	float time = 0;
	Unigine::Math::Vec3 prev_binormal;

	// modes
	enum class MODE
	{
		POSITION_ONLY,		// position
		POSITION_FRENET,	// position + rotation (based on Frenet Frame)
		POSITION_RMF,		// position + rotation (based on Rotation Minimizing Frame)
		POSITION_ROTATION,	// position + rotation (slerp)
		POSITION_SQUAD,		// position + rotation (SQUAD)
		FINAL,				// position + rotation (SQUAD) + velocity
	} mode = MODE::POSITION_ONLY;
	Unigine::WidgetLabelPtr label;
};

#endif // __APP_WORLD_LOGIC_H__
