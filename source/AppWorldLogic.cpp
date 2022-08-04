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

using namespace Unigine;
using namespace Math;

AppWorldLogic::AppWorldLogic()
{}

AppWorldLogic::~AppWorldLogic()
{}

int AppWorldLogic::init()
{
	return 1;
}

int AppWorldLogic::update()
{
	return 1;
}

int AppWorldLogic::postUpdate()
{
	return 1;
}

int AppWorldLogic::updatePhysics()
{
	return 1;
}

int AppWorldLogic::shutdown()
{
	return 1;
}

int AppWorldLogic::save(const Unigine::StreamPtr &stream)
{
	UNIGINE_UNUSED(stream);
	return 1;
}

int AppWorldLogic::restore(const Unigine::StreamPtr &stream)
{
	UNIGINE_UNUSED(stream);
	return 1;
}
