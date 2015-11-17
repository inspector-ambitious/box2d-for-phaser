/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

/**
 * Friction mixing law. The idea is to allow either fixture to
 * drive the restitution to zero. For example, anything slides
 * on ice.
 * @export
 * @return {number}
 * @param {number} friction1
 * @param {number} friction2
 */
box2d.b2MixFriction = function (friction1, friction2)
{
	return box2d.b2Sqrt(friction1 * friction2);
}

/**
 * Restitution mixing law. The idea is allow for anything to
 * bounce off an inelastic surface. For example, a superball
 * bounces on anything.
 * @export
 * @return {number}
 * @param {number} restitution1
 * @param {number} restitution2
 */
box2d.b2MixRestitution = function (restitution1, restitution2)
{
	return restitution1 > restitution2 ? restitution1 : restitution2;
}

/**
 * A contact edge is used to connect bodies and contacts
 * together in a contact graph where each body is a node and
 * each contact is an edge. A contact edge belongs to a doubly
 * linked list maintained in each attached body. Each contact
 * has two contact nodes, one for each attached body.
 * @export
 * @constructor
 */
box2d.b2ContactEdge = function ()
{
	/**
	 * @export
	 * @type {box2d.b2Body}
	 */
	this.other = null; ///< provides quick access to the other body attached.
	/**
	 * @export
	 * @type {box2d.b2Contact}
	 */
	this.contact = null; ///< the contact

};


/**
 * The class manages contact between two shapes. A contact
 * exists for each overlapping AABB in the broad-phase (except
 * if filtered). Therefore a contact object may exist that has
 * no contact points.
 * @export
 * @constructor
 */
box2d.b2Contact = function (shapeTypeA, shapeTypeB)
{
	this.m_nodeA = new box2d.b2ContactEdge();
	this.m_nodeB = new box2d.b2ContactEdge();
	this.m_manifold = new box2d.b2Manifold();
	this.m_oldManifold = new box2d.b2Manifold();

	this.shapeTypeA = shapeTypeA;
	this.shapeTypeB = shapeTypeB;
	/**
	 * @export
	 * @type {boolean}
	 */
	this.m_flag_islandFlag = false;		/// Used when crawling contact graph when forming islands.
	/**
	 * @export
	 * @type {boolean}
	 */
	this.m_flag_touchingFlag = false;	/// Set when the shapes are touching.
	/**
	 * @export
	 * @type {boolean}
	 */
	this.m_flag_enabledFlag = false;	/// This contact can be disabled (by user)
	/**
	 * @export
	 * @type {boolean}
	 */
	this.m_flag_filterFlag = false;		/// This contact needs filtering because a fixture filter was changed.
	/**
	 * @export
	 * @type {boolean}
	 */
	this.m_flag_bulletHitFlag = false;	/// This bullet contact had a TOI event
	/**
	 * @export
	 * @type {boolean}
	 */
	this.m_flag_toiFlag = false;		/// This contact has a valid TOI in m_toi

	/**
	 * World pool and list pointers.
	 * @export
	 * @type {box2d.b2Contact}
	 */
	this.m_prev = null;
	/**
	 * @export
	 * @type {box2d.b2Contact}
	 */
	this.m_next = null;

	/**
	 * @export
	 * @type {box2d.b2Fixture}
	 */
	this.m_fixtureA = null;
	/**
	 * @export
	 * @type {box2d.b2Fixture}
	 */
	this.m_fixtureB = null;

	/**
	 * @export
	 * @type {number}
	 */
	this.m_indexA = 0;
	/**
	 * @export
	 * @type {number}
	 */
	this.m_indexB = 0;


	/**
	 * @export
	 * @type {number}
	 */
	this.m_toiCount = 0;
	/**
	 * @export
	 * @type {number}
	 */
	this.m_toi = 1.0;

	/**
	 * @export
	 * @type {number}
	 */
	this.m_friction = 0.0;
	/**
	 * @export
	 * @type {number}
	 */
	this.m_restitution = 0.0;

	/**
	 * @export
	 * @type {number}
	 */
	this.m_tangentSpeed = 0.0;

}

box2d.b2Contact.solveTOI_minContact = new box2d.b2Contact(); 

/**
 * Get the contact manifold. Do not modify the manifold unless
 * you understand the internals of Box2D.
 * @export
 * @return {box2d.b2Manifold}
 */
box2d.b2Contact.prototype.GetManifold = function ()
{
	return this.m_manifold;
}

/**
 * Get the world manifold.
 * @export
 * @return {void}
 * @param {box2d.b2WorldManifold} worldManifold
 */
box2d.b2Contact.prototype.GetWorldManifold = function (worldManifold)
{
	var bodyA = this.m_fixtureA.GetBody();
	var bodyB = this.m_fixtureB.GetBody();
	var shapeA = this.m_fixtureA.GetShape();
	var shapeB = this.m_fixtureB.GetShape();
	worldManifold.Initialize(this.m_manifold, bodyA.GetTransform(), shapeA.m_radius, bodyB.GetTransform(), shapeB.m_radius);
}

/**
 * Is this contact touching?
 * @export
 * @return {boolean}
 */
box2d.b2Contact.prototype.IsTouching = function ()
{
	return this.m_flag_touchingFlag;
}

/**
 * Enable/disable this contact. This can be used inside the
 * pre-solve contact listener. The contact is only disabled for
 * the current time step (or sub-step in continuous collisions).
 * @export
 * @return {void}
 * @param {boolean} flag
 */
box2d.b2Contact.prototype.SetEnabled = function (flag)
{
	this.m_flag_enabledFlag = flag;
}

/**
 * Has this contact been disabled?
 * @export
 * @return {boolean}
 */
box2d.b2Contact.prototype.IsEnabled = function ()
{
	return this.m_flag_enabledFlag;
}

/**
 * Get the next contact in the world's contact list.
 * @export
 * @return {box2d.b2Contact}
 */
box2d.b2Contact.prototype.GetNext = function ()
{
	return this.m_next;
}

/**
 * Get fixture A in this contact.
 * @export
 * @return {box2d.b2Fixture}
 */
box2d.b2Contact.prototype.GetFixtureA = function ()
{
	return this.m_fixtureA;
}

/**
 * @export
 * @return {number}
 */
box2d.b2Contact.prototype.GetChildIndexA = function ()
{
	return this.m_indexA;
}

/**
 * Get fixture B in this contact.
 * @export
 * @return {box2d.b2Fixture}
 */
box2d.b2Contact.prototype.GetFixtureB = function ()
{
	return this.m_fixtureB;
}

/**
 * @export
 * @return {number}
 */
box2d.b2Contact.prototype.GetChildIndexB = function ()
{
	return this.m_indexB;
}

/**
 * Evaluate this contact with your own manifold and transforms.
 * @export
 * @return {void}
 * @param {box2d.b2Manifold} manifold
 * @param {box2d.b2Transform} xfA
 * @param {box2d.b2Transform} xfB
 */
box2d.b2Contact.prototype.Evaluate = function (manifold, xfA, xfB)
{
	var shapeA = this.m_fixtureA.GetShape();
	var shapeB = this.m_fixtureB.GetShape();
	
	// circle contact
	if (this.shapeTypeA === box2d.b2ShapeType.e_circleShape && this.shapeTypeB === box2d.b2ShapeType.e_circleShape) {
		if (BOX2D_ENABLE_ASSERTS) { box2d.b2Assert(shapeA instanceof box2d.b2CircleShape); }
		if (BOX2D_ENABLE_ASSERTS) { box2d.b2Assert(shapeB instanceof box2d.b2CircleShape); }
		box2d.b2CollideCircles(
			manifold, 
			(shapeA instanceof box2d.b2CircleShape)? shapeA : null, xfA, 
			(shapeB instanceof box2d.b2CircleShape)? shapeB : null, xfB);
	}
	
	// polygon and circle contact
	if (this.shapeTypeA === box2d.b2ShapeType.e_polygonShape && this.shapeTypeB === box2d.b2ShapeType.e_circleShape) {
		if (BOX2D_ENABLE_ASSERTS) { box2d.b2Assert(shapeA instanceof box2d.b2PolygonShape); }
		if (BOX2D_ENABLE_ASSERTS) { box2d.b2Assert(shapeB instanceof box2d.b2CircleShape); }
		box2d.b2CollidePolygonAndCircle(
			manifold, 
			(shapeA instanceof box2d.b2PolygonShape)? shapeA : null, xfA, 
			(shapeB instanceof box2d.b2CircleShape)? shapeB : null, xfB);
	}
	
	
	// polygon contact
	if (this.shapeTypeA === box2d.b2ShapeType.e_polygonShape && this.shapeTypeB === box2d.b2ShapeType.e_polygonShape) {
		if (BOX2D_ENABLE_ASSERTS) { box2d.b2Assert(shapeA instanceof box2d.b2PolygonShape); }
		if (BOX2D_ENABLE_ASSERTS) { box2d.b2Assert(shapeB instanceof box2d.b2PolygonShape); }
		box2d.b2CollidePolygons(
			manifold, 
			(shapeA instanceof box2d.b2PolygonShape)? shapeA : null, xfA, 
			(shapeB instanceof box2d.b2PolygonShape)? shapeB : null, xfB);
	}
	

	// edge and circle contact
	if (this.shapeTypeA === box2d.b2ShapeType.e_edgeShape && this.shapeTypeB === box2d.b2ShapeType.e_circleShape) {
		if (BOX2D_ENABLE_ASSERTS) { box2d.b2Assert(shapeA instanceof box2d.b2EdgeShape); }
		if (BOX2D_ENABLE_ASSERTS) { box2d.b2Assert(shapeB instanceof box2d.b2CircleShape); }
		box2d.b2CollideEdgeAndCircle(
			manifold, 
			(shapeA instanceof box2d.b2EdgeShape)? shapeA : null, xfA, 
			(shapeB instanceof box2d.b2CircleShape)? shapeB : null, xfB);
	}
	
	// edge and polygon contact
	if (this.shapeTypeA === box2d.b2ShapeType.e_edgeShape && this.shapeTypeB === box2d.b2ShapeType.e_polygonShape) {
		if (BOX2D_ENABLE_ASSERTS) { box2d.b2Assert(shapeA instanceof box2d.b2EdgeShape); }
		if (BOX2D_ENABLE_ASSERTS) { box2d.b2Assert(shapeB instanceof box2d.b2PolygonShape); }
		box2d.b2CollideEdgeAndPolygon(
			manifold, 
			(shapeA instanceof box2d.b2EdgeShape)? shapeA : null, xfA, 
			(shapeB instanceof box2d.b2PolygonShape)? shapeB : null, xfB);
	}
	

	// chain and circle contact
	if (this.shapeTypeA === box2d.b2ShapeType.e_chainShape && this.shapeTypeB === box2d.b2ShapeType.e_circleShape) {
		if (BOX2D_ENABLE_ASSERTS) { box2d.b2Assert(shapeA instanceof box2d.b2ChainShape); }
		if (BOX2D_ENABLE_ASSERTS) { box2d.b2Assert(shapeB instanceof box2d.b2CircleShape); }
		/*box2d.b2ChainShape*/ var chain = (shapeA instanceof box2d.b2ChainShape)? shapeA : null;
		/*box2d.b2EdgeShape*/ var edge = box2d.b2ChainAndCircleContact.prototype.Evaluate.s_edge;
		chain.GetChildEdge(edge, this.m_indexA);
		box2d.b2CollideEdgeAndCircle(
			manifold, 
			edge, xfA, 
			(shapeB instanceof box2d.b2CircleShape)? shapeB : null, xfB);
	}
	
	if (this.shapeTypeA === box2d.b2ShapeType.e_chainShape && this.shapeTypeB === box2d.b2ShapeType.e_polygonShape) {
		if (BOX2D_ENABLE_ASSERTS) { box2d.b2Assert(shapeA instanceof box2d.b2ChainShape); }
		if (BOX2D_ENABLE_ASSERTS) { box2d.b2Assert(shapeB instanceof box2d.b2PolygonShape); }
		/*box2d.b2ChainShape*/ var chain = (shapeA instanceof box2d.b2ChainShape)? shapeA : null;
		/*box2d.b2EdgeShape*/ var edge = box2d.b2ChainAndPolygonContact.prototype.Evaluate.s_edge;
		chain.GetChildEdge(edge, this.m_indexA);
		box2d.b2CollideEdgeAndPolygon(
			manifold, 
			edge, xfA, 
			(shapeB instanceof box2d.b2PolygonShape)? shapeB : null, xfB);
	}
		
}

/**
 * Flag this contact for filtering. Filtering will occur the
 * next time step.
 * @export
 * @return {void}
 */
box2d.b2Contact.prototype.FlagForFiltering = function ()
{
	this.m_flag_filterFlag = true;
}

/**
 * Override the default friction mixture. You can call this in
 * box2d.b2ContactListener::PreSolve.
 * This value persists until set or reset.
 * @export
 * @return {void}
 * @param {number} friction
 */
box2d.b2Contact.prototype.SetFriction = function (friction)
{
	this.m_friction = friction;
}

/**
 * Get the friction.
 * @export
 * @return {number}
 */
box2d.b2Contact.prototype.GetFriction = function ()
{
	return this.m_friction;
}

/**
 * Reset the friction mixture to the default value.
 * @export
 * @return {void}
 */
box2d.b2Contact.prototype.ResetFriction = function ()
{
	this.m_friction = box2d.b2MixFriction(this.m_fixtureA.m_friction, this.m_fixtureB.m_friction);
}

/**
 * Override the default restitution mixture. You can call this
 * in box2d.b2ContactListener::PreSolve.
 * The value persists until you set or reset.
 * @export
 * @return {void}
 * @param {number} restitution
 */
box2d.b2Contact.prototype.SetRestitution = function (restitution)
{
	this.m_restitution = restitution;
}

/**
 * Get the restitution.
 * @export
 * @return {number}
 */
box2d.b2Contact.prototype.GetRestitution = function ()
{
	return this.m_restitution;
}

/**
 * Reset the restitution to the default value.
 * @export
 * @return {void}
 */
box2d.b2Contact.prototype.ResetRestitution = function ()
{
	this.m_restitution = box2d.b2MixRestitution(this.m_fixtureA.m_restitution, this.m_fixtureB.m_restitution);
}

/**
 * Set the desired tangent speed for a conveyor belt behavior.
 * In meters per second.
 * @export
 * @return {void}
 * @param {number} speed
 */
box2d.b2Contact.prototype.SetTangentSpeed = function (speed)
{
	this.m_tangentSpeed = speed;
}

/**
 * Get the desired tangent speed. In meters per second.
 * @export
 * @return {number}
 */
box2d.b2Contact.prototype.GetTangentSpeed = function ()
{
	return this.m_tangentSpeed;
}

/**
 * @export
 * @return {void}
 * @param {box2d.b2Fixture} fixtureA
 * @param {box2d.b2Fixture} fixtureB
 */
box2d.b2Contact.prototype.Reset = function (fixtureA, indexA, fixtureB, indexB)
{
	this.m_flag_islandFlag = false;
	this.m_flag_touchingFlag = false;
	this.m_flag_enabledFlag = true;
	this.m_flag_filterFlag = false;
	this.m_flag_bulletHitFlag = false;
	this.m_flag_toiFlag = false;

	this.m_fixtureA = fixtureA;
	this.m_fixtureB = fixtureB;

	this.m_indexA = indexA;
	this.m_indexB = indexB;

	this.m_manifold.pointCount = 0;

	this.m_prev = null;
	this.m_next = null;

	this.m_nodeA.contact = null;
	this.m_nodeA.other = null;

	this.m_nodeB.contact = null;
	this.m_nodeB.other = null;

	this.m_toiCount = 0;

	this.m_friction = box2d.b2MixFriction(this.m_fixtureA.m_friction, this.m_fixtureB.m_friction);
	this.m_restitution = box2d.b2MixRestitution(this.m_fixtureA.m_restitution, this.m_fixtureB.m_restitution);
}

/**
 * Update the contact manifold and touching status.
 * Note: do not assume the fixture AABBs are overlapping or are
 * valid.
 * @export
 * @return {void}
 * @param {box2d.b2ContactListener} listener
 */
box2d.b2Contact.prototype.Update = function (listener)
{
	var tManifold = this.m_oldManifold;
	this.m_oldManifold = this.m_manifold;
	this.m_manifold = tManifold;

	// Re-enable this contact.
	this.m_flag_enabledFlag = true;

	var touching = false;
	var wasTouching = this.m_flag_touchingFlag;

	var sensorA = this.m_fixtureA.IsSensor();
	var sensorB = this.m_fixtureB.IsSensor();
	var sensor = sensorA || sensorB;

	var bodyA = this.m_fixtureA.GetBody();
	var bodyB = this.m_fixtureB.GetBody();
	var xfA = bodyA.GetTransform();
	var xfB = bodyB.GetTransform();

//	var aabbOverlap = box2d.b2TestOverlap_AABB(this.m_fixtureA.GetAABB(0), this.m_fixtureB.GetAABB(0));

	// Is this contact a sensor?
	if (sensor)
	{
//		if (aabbOverlap)
//		{
			var shapeA = this.m_fixtureA.GetShape();
			var shapeB = this.m_fixtureB.GetShape();
			touching = box2d.b2TestOverlap_Shape(shapeA, this.m_indexA, shapeB, this.m_indexB, xfA, xfB);
//		}

		// Sensors don't generate manifolds.
		this.m_manifold.pointCount = 0;
	}
	else
	{
//		if (aabbOverlap)
//		{
			this.Evaluate(this.m_manifold, xfA, xfB);
			touching = this.m_manifold.pointCount > 0;

			// Match old contact ids to new contact ids and copy the
			// stored impulses to warm start the solver.
			for (var i = 0; i < this.m_manifold.pointCount; ++i)
			{
				var mp2 = this.m_manifold.points[i];
				mp2.normalImpulse = 0;
				mp2.tangentImpulse = 0;
				var id2 = mp2.id;

				for (var j = 0; j < this.m_oldManifold.pointCount; ++j)
				{
					var mp1 = this.m_oldManifold.points[j];

					if (mp1.id.key === id2.key)
					{
						mp2.normalImpulse = mp1.normalImpulse;
						mp2.tangentImpulse = mp1.tangentImpulse;
						break;
					}
				}
			}
//		}
//		else
//		{
//			this.m_manifold.pointCount = 0;
//		}

		if (touching !== wasTouching)
		{
			bodyA.SetAwake(true);
			bodyB.SetAwake(true);
		}
	}

	this.m_flag_touchingFlag = touching;

	if (!wasTouching && touching && listener)
	{
		listener.BeginContact(this);
	}

	if (wasTouching && !touching && listener)
	{
		listener.EndContact(this);
	}

	if (!sensor && touching && listener)
	{
		listener.PreSolve(this, this.m_oldManifold);
	}
}

/**
 * @export
 * @return {number}
 * @param {box2d.b2Sweep} sweepA
 * @param {box2d.b2Sweep} sweepB
 */
box2d.b2Contact.prototype.ComputeTOI = function (sweepA, sweepB)
{
	var input = box2d.b2Contact.prototype.ComputeTOI.s_input;
	input.proxyA.SetShape(this.m_fixtureA.GetShape(), this.m_indexA);
	input.proxyB.SetShape(this.m_fixtureB.GetShape(), this.m_indexB);
	input.sweepA.Copy(sweepA);
	input.sweepB.Copy(sweepB);
	input.tMax = box2d.b2_linearSlop;

	var output = box2d.b2Contact.prototype.ComputeTOI.s_output;

	box2d.b2TimeOfImpact(output, input);

	return output.t;
}
box2d.b2Contact.prototype.ComputeTOI.s_input = new box2d.b2TOIInput();
box2d.b2Contact.prototype.ComputeTOI.s_output = new box2d.b2TOIOutput();
