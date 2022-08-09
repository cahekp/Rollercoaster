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


#include "AppWorldLogic.h"
#include <UnigineMathLib.h>
#include <UnigineWorld.h>
#include <UnigineVisualizer.h>
#include <UnigineGame.h>
#include <UnigineInput.h>
#include <UnigineConsole.h>
#include "Math.h"

using namespace Unigine;
using namespace Math;

int AppWorldLogic::init()
{
	// get path points (and rotations)
	points_pos.clear();
	points_rot.clear();
	NodePtr n = World::getNodeByName("points");
	if (n)
	{
		for (int i = 0; i < n->getNumChildren(); i++)
		{
			NodePtr nc = n->getChild(i);
			points_pos.append(nc->getWorldPosition());
			points_rot.append(nc->getWorldRotation());
		}
	}

	// calculate length of curves
	lengths.clear();
	int points_count = points_pos.size();
	for (int j = 0; j < points_count; j++)
	{
		int j_prev = (j - 1 < 0) ? (points_count - 1) : j - 1;
		int j_cur = j;
		int j_next = (j + 1) % points_count;
		int j_next_next = (j + 2) % points_count;

		const Vec3 &p0 = points_pos[j_prev];
		const Vec3 &p1 = points_pos[j_cur];
		const Vec3 &p2 = points_pos[j_next];
		const Vec3 &p3 = points_pos[j_next_next];

		lengths.append(getLengthCatmullRomUniform(p0, p1, p2, p3));
	}

	// calculate start binormal (uses in Rotation Minimizing Frame)
	{
		const Vec3 &p0 = points_pos[points_count - 1];
		const Vec3 &p1 = points_pos[0];
		const Vec3 &p2 = points_pos[1];
		const Vec3 &p3 = points_pos[2];
		Vec3 tangent = normalize(catmullRomUniformTangent(p0, p1, p2, p3, 0));
		prev_binormal = normalize(cross(tangent, Vec3_up));
	}

	// add help
	label = WidgetLabel::create();
	label->setPosition(10, 10);
	label->setFontOutline(1);
	label->setFontSize(24);
	label->setFontRich(1);
	Gui::get()->addChild(label, Gui::ALIGN_OVERLAP);

	return 1;
}

int AppWorldLogic::update()
{
	// simple visualization
	draw_path();

	// change modes
	if (Input::isKeyDown(Input::KEY_SPACE) && !Console::isActive())
	{
		// loop
		if (mode == MODE::FINAL)
			mode = (MODE)0;
		else
			mode = (MODE)((int)mode + 1);
	}

	// update text
	label->setText(String::format(
		"Press SPACE to change mode<br>"
		"<font color=%s>1. Position</font><br>"
		"<font color=%s>2. Position and Rotation (based on Frenet Frame)</font><br>"
		"<font color=%s>3. Position and Rotation (based on Rotation Minimizing Frame)</font><br>"
		"<font color=%s>4. Position and Rotation (slerp - spherical interpolation)</font><br>"
		"<font color=%s>5. Position and Rotation (SQUAD - spherical quadrangle interpolation)</font><br>"
		"<font color=%s>6. Position and Rotation (SQUAD) + velocity</font><br>",
		mode == MODE::POSITION_ONLY ? "ffffff" : "aaaaaa",
		mode == MODE::POSITION_FRENET ? "ffffff" : "aaaaaa",
		mode == MODE::POSITION_RMF ? "ffffff" : "aaaaaa",
		mode == MODE::POSITION_ROTATION ? "ffffff" : "aaaaaa",
		mode == MODE::POSITION_SQUAD ? "ffffff" : "aaaaaa",
		mode == MODE::FINAL ? "ffffff" : "aaaaaa"
	));

	// update selected mode
	switch (mode)
	{
		case MODE::POSITION_ONLY: update_position_only(); break;
		case MODE::POSITION_FRENET: update_position_frenet(); break;
		case MODE::POSITION_RMF: update_position_rmf(); break;
		case MODE::POSITION_ROTATION: update_position_rotation(); break;
		case MODE::POSITION_SQUAD: update_position_squad(); break;
		case MODE::FINAL: update_final(); break;
	}

	return 1;
}

int AppWorldLogic::shutdown()
{
	label.deleteLater();
	return 1;
}

void AppWorldLogic::draw_path() const
{
	// draw spline
	Visualizer::setEnabled(true);
	int points_count = points_pos.size();
	for (int j = 0; j < points_count; j++)
	{
		int j_prev = (j - 1 < 0) ? (points_count - 1) : j - 1;
		int j_cur = j;
		int j_next = (j + 1) % points_count;
		int j_next_next = (j + 2) % points_count;

		const Vec3 &p0 = points_pos[j_prev];
		const Vec3 &p1 = points_pos[j_cur];
		const Vec3 &p2 = points_pos[j_next];
		const Vec3 &p3 = points_pos[j_next_next];

		// draw curve
		Vec3 start = catmullRomUniform(p0, p1, p2, p3, 0);
		for (int i = 1; i < quality; i++)
		{
			Vec3 end = catmullRomUniform(p0, p1, p2, p3, float(i) / (quality - 1));
			Visualizer::renderLine3D(start, end, vec4(1,1,1,1));
			start = end;
		}
	}
}

void AppWorldLogic::update_time(float speed)
{
	time += speed * Game::getIFps();
	if (time >= 1.0f)
	{
		points_index = (points_index + ftoi(time)) % points_pos.size(); // loop
		time = Math::frac(time);
	}
}

VectorStack<Vec3, 4> AppWorldLogic::get_current_points() const
{
	int points_count = points_pos.size();
	int i_prev = (points_index - 1 < 0) ? (points_count - 1) : points_index - 1;
	int i_cur = points_index;
	int i_next = (points_index + 1) % points_count;
	int i_next_next = (points_index + 2) % points_count;
	
	VectorStack<Vec3, 4> result;
	result.append(points_pos[i_prev]);
	result.append(points_pos[i_cur]);
	result.append(points_pos[i_next]);
	result.append(points_pos[i_next_next]);
	return result;
}

VectorStack<quat, 4> AppWorldLogic::get_current_quats() const
{
	int points_count = points_pos.size();
	int i_prev = (points_index - 1 < 0) ? (points_count - 1) : points_index - 1;
	int i_cur = points_index;
	int i_next = (points_index + 1) % points_count;
	int i_next_next = (points_index + 2) % points_count;

	VectorStack<quat, 4> result;
	result.append(points_rot[i_prev]);
	result.append(points_rot[i_cur]);
	result.append(points_rot[i_next]);
	result.append(points_rot[i_next_next]);
	return result;
}

void AppWorldLogic::update_position_only()
{
	// update time and get current control points
	update_time();
	VectorStack<Vec3, 4> p = get_current_points();

	// spline calculations
	Vec3 pos = catmullRomUniform(p[0], p[1], p[2], p[3], time);
	
	// change camera position
	Game::getPlayer()->setWorldPosition(pos);
}

void AppWorldLogic::update_position_frenet()
{
	// update time and get current control points
	update_time();
	VectorStack<Vec3, 4> p = get_current_points();

	// spline calculations
	Mat4 t = catmullRomUniformFrenet(p[0], p[1], p[2], p[3], time);

	// change camera position and rotation
	Game::getPlayer()->setWorldTransform(t * rotateX(90.0f));
}

void AppWorldLogic::update_position_rmf()
{
	// update time and get current control points
	update_time();
	VectorStack<Vec3, 4> p = get_current_points();

	// spline calculations
	Mat4 t = catmullRomUniformRMF(prev_binormal, p[0], p[1], p[2], p[3], time);
	prev_binormal = t.getColumn3(0);

	// change camera position and rotation
	Game::getPlayer()->setWorldTransform(t * rotateX(90.0f));
}

void AppWorldLogic::update_position_rotation()
{
	// update time and get current control points
	update_time();
	VectorStack<Vec3, 4> p = get_current_points();
	VectorStack<quat, 4> q = get_current_quats();

	// spline calculations
	Vec3 pos = catmullRomUniform(p[0], p[1], p[2], p[3], time);
	quat rot = slerp(q[1], q[2], time) * quat(1, 0, 0, 90);

	// change camera position
	Game::getPlayer()->setWorldPosition(pos);
	Game::getPlayer()->setWorldRotation(rot, true);
}

void AppWorldLogic::update_position_squad()
{
	// update time and get current control points
	update_time();
	VectorStack<Vec3, 4> p = get_current_points();
	VectorStack<quat, 4> q = get_current_quats();

	// spline calculations
	Vec3 pos = catmullRomUniform(p[0], p[1], p[2], p[3], time);
	quat rot = squad(q[0], q[1], q[2], q[3], time) * quat(1, 0, 0, 90);

	// change camera position
	Game::getPlayer()->setWorldPosition(pos);
	Game::getPlayer()->setWorldRotation(rot, true);
}

void AppWorldLogic::update_final()
{
	// update time
	float speed = 100.0f / lengths[points_index];
	update_time(speed);

	// get current control points
	VectorStack<Vec3, 4> p = get_current_points();
	VectorStack<quat, 4> q = get_current_quats();

	// spline calculations
	Vec3 pos = catmullRomUniform(p[0], p[1], p[2], p[3], time);
	quat rot = squad(q[0], q[1], q[2], q[3], time) * quat(1, 0, 0, 90);

	// change camera position
	Game::getPlayer()->setWorldPosition(pos);
	Game::getPlayer()->setWorldRotation(rot, true);
}
