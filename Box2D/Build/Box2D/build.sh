./node_modules/uglify-js/bin/uglifyjs ../../Box2D/Box2D.js  ../../../Box2D/Box2D/Common/b2Settings.js ../../../Box2D/Box2D/Common/b2Draw.js ../../../Box2D/Box2D/Common/b2GrowableStack.js ../../../Box2D/Box2D/Common/b2Math.js ../../../Box2D/Box2D/Collision/b2Distance.js ../../../Box2D/Box2D/Collision/b2Collision.js ../../../Box2D/Box2D/Collision/b2CollideCircle.js ../../../Box2D/Box2D/Collision/b2CollideEdge.js ../../../Box2D/Box2D/Collision/b2CollidePolygon.js ../../../Box2D/Box2D/Collision/b2DynamicTree.js ../../../Box2D/Box2D/Collision/b2BroadPhase.js ../../../Box2D/Box2D/Collision/Shapes/b2Shape.js ../../../Box2D/Box2D/Collision/Shapes/b2CircleShape.js ../../../Box2D/Box2D/Collision/Shapes/b2EdgeShape.js ../../../Box2D/Box2D/Collision/Shapes/b2ChainShape.js ../../../Box2D/Box2D/Collision/Shapes/b2PolygonShape.js ../../../Box2D/Box2D/Common/b2Timer.js ../../../Box2D/Box2D/Collision/b2TimeOfImpact.js ../../../Box2D/Box2D/Dynamics/b2Fixture.js ../../../Box2D/Box2D/Dynamics/b2Body.js ../../../Box2D/Box2D/Dynamics/b2TimeStep.js ../../../Box2D/Box2D/Dynamics/b2WorldCallbacks.js ../../../Box2D/Box2D/Dynamics/Contacts/b2Contact.js ../../../Box2D/Box2D/Dynamics/Contacts/b2ChainAndCircleContact.js ../../../Box2D/Box2D/Dynamics/Contacts/b2ChainAndPolygonContact.js ../../../Box2D/Box2D/Dynamics/Contacts/b2CircleContact.js ../../../Box2D/Box2D/Dynamics/Contacts/b2ContactFactory.js ../../../Box2D/Box2D/Dynamics/b2ContactManager.js ../../../Box2D/Box2D/Dynamics/Contacts/b2EdgeAndCircleContact.js ../../../Box2D/Box2D/Dynamics/Contacts/b2EdgeAndPolygonContact.js ../../../Box2D/Box2D/Dynamics/Contacts/b2PolygonAndCircleContact.js ../../../Box2D/Box2D/Dynamics/Contacts/b2PolygonContact.js ../../../Box2D/Box2D/Dynamics/Contacts/b2ContactSolver.js ../../../Box2D/Box2D/Dynamics/b2Island.js ../../../Box2D/Box2D/Dynamics/Joints/b2Joint.js ../../../Box2D/Box2D/Dynamics/Joints/b2AreaJoint.js ../../../Box2D/Box2D/Dynamics/Joints/b2DistanceJoint.js ../../../Box2D/Box2D/Dynamics/Joints/b2FrictionJoint.js ../../../Box2D/Box2D/Dynamics/Joints/b2JointFactory.js ../../../Box2D/Box2D/Dynamics/b2World.js ../../../Box2D/Box2D/Dynamics/Joints/b2MotorJoint.js ../../../Box2D/Box2D/Dynamics/Joints/b2MouseJoint.js ../../../Box2D/Box2D/Dynamics/Joints/b2PrismaticJoint.js ../../../Box2D/Box2D/Dynamics/Joints/b2PulleyJoint.js ../../../Box2D/Box2D/Dynamics/Joints/b2RevoluteJoint.js ../../../Box2D/Box2D/Dynamics/Joints/b2GearJoint.js ../../../Box2D/Box2D/Dynamics/Joints/b2RopeJoint.js ../../../Box2D/Box2D/Dynamics/Joints/b2WeldJoint.js ../../../Box2D/Box2D/Dynamics/Joints/b2WheelJoint.js ../../../Box2D/Box2D/Rope/b2Rope.js -b -ns -c unused=false,sequences=false,hoist_funs=false,keep_fargs=true,keep_fnames=true,booleans=false > box2d-html5.js 
cat box2d-html5.js | awk '{ gsub("function\\(", "function anon" NR "("); print $0 }' > box2d-html5-deanon.js
cp box2d-html5.js ~/Repos/X/www/js/
cp box2d-html5-deanon.js ~/Repos/X/www/js/