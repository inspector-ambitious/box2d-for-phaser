/*
* Copyright (c) 2014 Google, Inc.
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

//#if B2_ENABLE_PARTICLE

goog.provide('box2d.Testbed.CornerCase');

goog.require('box2d.Testbed.Test');

/**
 * @export 
 * @constructor 
 * @extends {box2d.Testbed.Test} 
 * @param {HTMLCanvasElement} canvas 
 * @param {box2d.Testbed.Settings} settings 
 */
box2d.Testbed.CornerCase = function (canvas, settings)
{
	box2d.Testbed.Test.call(this, canvas, settings); // base class constructor

	{
		var bd = new box2d.b2BodyDef();
		var ground = this.m_world.CreateBody(bd);

		// Construct a pathological corner intersection out of many
		// polygons to ensure there's no issue with particle oscillation
		// from many fixture contact impulses at the corner

		// left edge
		{
			var shape = new box2d.b2PolygonShape();
			var vertices = [
				new box2d.b2Vec2(-20.0, 30.0),
				new box2d.b2Vec2(-20.0, 0.0),
				new box2d.b2Vec2(-25.0, 0.0),
				new box2d.b2Vec2(-25.0, 30.0)
			];
			shape.Set(vertices);
			ground.CreateFixture(shape, 0.0);
		}

		var x, y;
		var yrange=30.0, ystep = yrange/10.0, xrange=20.0, xstep=xrange/2.0;

		{
			var shape = new box2d.b2PolygonShape();
			var vertices = [
				new box2d.b2Vec2(-25.0, 0.0),
				new box2d.b2Vec2(20.0, 15.0),
				new box2d.b2Vec2(25.0, 0.0)
			];
			shape.Set(vertices);
			ground.CreateFixture(shape, 0.0);
		}

		for (x = -xrange; x < xrange; x += xstep)
		{
			var shape = new box2d.b2PolygonShape();
			var vertices = [
				new box2d.b2Vec2(-25.0, 0.0),
				new box2d.b2Vec2(x, 15.0),
				new box2d.b2Vec2(x+xstep, 15.0)
			];
			shape.Set(vertices);
			ground.CreateFixture(shape, 0.0);
		}

		for (y = 0.0; y < yrange; y += ystep)
		{
			var shape = new box2d.b2PolygonShape();
			var vertices = [
				new box2d.b2Vec2(25.0, y),
				new box2d.b2Vec2(25.0, y+ystep),
				new box2d.b2Vec2(20.0, 15.0)
			];
			shape.Set(vertices);
			ground.CreateFixture(shape, 0.0);
		}

	}

	this.m_particleSystem.SetRadius(1.0);
	var particleType = box2d.Testbed.TestMain.GetParticleParameterValue();

	{
		var shape = new box2d.b2CircleShape();
		shape.m_p.Set(0, 35);
		shape.m_radius = 12;
		var pd = new box2d.b2ParticleGroupDef();
		pd.flags = particleType;
		pd.shape = shape;
		var group = this.m_particleSystem.CreateParticleGroup(pd);
		if (pd.flags & box2d.b2ParticleFlag.b2_colorMixingParticle)
		{
			this.ColorParticleGroup(group, 0);
		}
	}
}

goog.inherits(box2d.Testbed.CornerCase, box2d.Testbed.Test);

/** 
 * @export 
 * @return {box2d.Testbed.Test} 
 * @param {HTMLCanvasElement} canvas 
 * @param {box2d.Testbed.Settings} settings 
 */
box2d.Testbed.CornerCase.Create = function (canvas, settings)
{
	return new box2d.Testbed.CornerCase(canvas, settings);
}

//#endif

