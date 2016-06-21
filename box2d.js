/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
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
var Box2D = {};

(function (a2j, undefined) {

   if(!(Object.prototype.defineProperty instanceof Function)
      && Object.prototype.__defineGetter__ instanceof Function
      && Object.prototype.__defineSetter__ instanceof Function)
   {
      Object.defineProperty = function(obj, p, cfg) {
         if(cfg.get instanceof Function)
            obj.__defineGetter__(p, cfg.get);
         if(cfg.set instanceof Function)
            obj.__defineSetter__(p, cfg.set);
      }
   }
   
   function emptyFn() {};
   a2j.inherit = function(cls, base) {
      var tmpCtr = cls;
      emptyFn.prototype = base.prototype;
      cls.prototype = new emptyFn;
      cls.prototype.constructor = tmpCtr;
   };
   
   a2j.generateCallback = function generateCallback(context, cb) {
      return function () {
         cb.apply(context, arguments);
      };
   };
   
   a2j.NVector = function NVector(length) {
      if (length === undefined) length = 0;
      var tmp = new Array(length || 0);
      for (var i = 0; i < length; ++i)
      tmp[i] = 0;
      return tmp;
   };
   
   a2j.is = function is(o1, o2) {
      if (o1 === null) return false;
      if ((o2 instanceof Function) && (o1 instanceof o2)) return true;
      if ((o1.constructor.__implements != undefined) && (o1.constructor.__implements[o2])) return true;
      return false;
   };
   
   a2j.parseUInt = function(v) {
      return Math.abs(parseInt(v));
   }
   
})(Box2D);

//#TODO remove assignments from global namespace
var Vector = Array;
var Vector_a2j_Number = Box2D.NVector;
//package structure
if (typeof(Box2D) === "undefined") Box2D = {};
if (typeof(Box2D.Collision) === "undefined") Box2D.Collision = {};
if (typeof(Box2D.Collision.Shapes) === "undefined") Box2D.Collision.Shapes = {};
if (typeof(Box2D.Common) === "undefined") Box2D.Common = {};
if (typeof(Box2D.Common.Math) === "undefined") Box2D.Common.Math = {};
if (typeof(Box2D.Dynamics) === "undefined") Box2D.Dynamics = {};
if (typeof(Box2D.Dynamics.Contacts) === "undefined") Box2D.Dynamics.Contacts = {};
if (typeof(Box2D.Dynamics.Controllers) === "undefined") Box2D.Dynamics.Controllers = {};
if (typeof(Box2D.Dynamics.Joints) === "undefined") Box2D.Dynamics.Joints = {};
//pre-definitions
(function () {
   Box2D.Collision.IBroadPhase = 'Box2D.Collision.IBroadPhase';

   function AABB() {
      AABB.AABB.apply(this, arguments);
   };
   Box2D.Collision.AABB = AABB;

   function Bound() {
      Bound.Bound.apply(this, arguments);
   };
   Box2D.Collision.Bound = Bound;

   function BoundValues() {
      BoundValues.BoundValues.apply(this, arguments);
      if (this.constructor === BoundValues) this.BoundValues.apply(this, arguments);
   };
   Box2D.Collision.BoundValues = BoundValues;

   function Collision() {
      Collision.Collision.apply(this, arguments);
   };
   Box2D.Collision.Collision = Collision;

   function ContactID() {
      ContactID.ContactID.apply(this, arguments);
      if (this.constructor === ContactID) this.ContactID.apply(this, arguments);
   };
   Box2D.Collision.ContactID = ContactID;

   function ContactPoint() {
      ContactPoint.ContactPoint.apply(this, arguments);
   };
   Box2D.Collision.ContactPoint = ContactPoint;

   function Distance() {
      Distance.Distance.apply(this, arguments);
   };
   Box2D.Collision.Distance = Distance;

   function DistanceInput() {
      DistanceInput.DistanceInput.apply(this, arguments);
   };
   Box2D.Collision.DistanceInput = DistanceInput;

   function DistanceOutput() {
      DistanceOutput.DistanceOutput.apply(this, arguments);
   };
   Box2D.Collision.DistanceOutput = DistanceOutput;

   function DistanceProxy() {
      DistanceProxy.DistanceProxy.apply(this, arguments);
   };
   Box2D.Collision.DistanceProxy = DistanceProxy;

   function DynamicTree() {
      DynamicTree.DynamicTree.apply(this, arguments);
      if (this.constructor === DynamicTree) this.DynamicTree.apply(this, arguments);
   };
   Box2D.Collision.DynamicTree = DynamicTree;

   function DynamicTreeBroadPhase() {
      DynamicTreeBroadPhase.DynamicTreeBroadPhase.apply(this, arguments);
   };
   Box2D.Collision.DynamicTreeBroadPhase = DynamicTreeBroadPhase;

   function DynamicTreeNode() {
      DynamicTreeNode.DynamicTreeNode.apply(this, arguments);
   };
   Box2D.Collision.DynamicTreeNode = DynamicTreeNode;

   function DynamicTreePair() {
      DynamicTreePair.DynamicTreePair.apply(this, arguments);
   };
   Box2D.Collision.DynamicTreePair = DynamicTreePair;

   function Manifold() {
      Manifold.Manifold.apply(this, arguments);
      if (this.constructor === Manifold) this.Manifold.apply(this, arguments);
   };
   Box2D.Collision.Manifold = Manifold;

   function ManifoldPoint() {
      ManifoldPoint.ManifoldPoint.apply(this, arguments);
      if (this.constructor === ManifoldPoint) this.ManifoldPoint.apply(this, arguments);
   };
   Box2D.Collision.ManifoldPoint = ManifoldPoint;

   function Point() {
      Point.Point.apply(this, arguments);
   };
   Box2D.Collision.Point = Point;

   function RayCastInput() {
      RayCastInput.RayCastInput.apply(this, arguments);
      if (this.constructor === RayCastInput) this.RayCastInput.apply(this, arguments);
   };
   Box2D.Collision.RayCastInput = RayCastInput;

   function RayCastOutput() {
      RayCastOutput.RayCastOutput.apply(this, arguments);
   };
   Box2D.Collision.RayCastOutput = RayCastOutput;

   function Segment() {
      Segment.Segment.apply(this, arguments);
   };
   Box2D.Collision.Segment = Segment;

   function SeparationFunction() {
      SeparationFunction.SeparationFunction.apply(this, arguments);
   };
   Box2D.Collision.SeparationFunction = SeparationFunction;

   function Simplex() {
      Simplex.Simplex.apply(this, arguments);
      if (this.constructor === Simplex) this.Simplex.apply(this, arguments);
   };
   Box2D.Collision.Simplex = Simplex;

   function SimplexCache() {
      SimplexCache.SimplexCache.apply(this, arguments);
   };
   Box2D.Collision.SimplexCache = SimplexCache;

   function SimplexVertex() {
      SimplexVertex.SimplexVertex.apply(this, arguments);
   };
   Box2D.Collision.SimplexVertex = SimplexVertex;

   function TimeOfImpact() {
      TimeOfImpact.TimeOfImpact.apply(this, arguments);
   };
   Box2D.Collision.TimeOfImpact = TimeOfImpact;

   function TOIInput() {
      TOIInput.TOIInput.apply(this, arguments);
   };
   Box2D.Collision.TOIInput = TOIInput;

   function WorldManifold() {
      WorldManifold.WorldManifold.apply(this, arguments);
      if (this.constructor === WorldManifold) this.WorldManifold.apply(this, arguments);
   };
   Box2D.Collision.WorldManifold = WorldManifold;

   function ClipVertex() {
      ClipVertex.ClipVertex.apply(this, arguments);
   };
   Box2D.Collision.ClipVertex = ClipVertex;

   function Features() {
      Features.Features.apply(this, arguments);
   };
   Box2D.Collision.Features = Features;

   function CircleShape() {
      CircleShape.CircleShape.apply(this, arguments);
      if (this.constructor === CircleShape) this.CircleShape.apply(this, arguments);
   };
   Box2D.Collision.Shapes.CircleShape = CircleShape;

   function EdgeChainDef() {
      EdgeChainDef.EdgeChainDef.apply(this, arguments);
      if (this.constructor === EdgeChainDef) this.EdgeChainDef.apply(this, arguments);
   };
   Box2D.Collision.Shapes.EdgeChainDef = EdgeChainDef;

   function EdgeShape() {
      EdgeShape.EdgeShape.apply(this, arguments);
      if (this.constructor === EdgeShape) this.EdgeShape.apply(this, arguments);
   };
   Box2D.Collision.Shapes.EdgeShape = EdgeShape;

   function MassData() {
      MassData.MassData.apply(this, arguments);
   };
   Box2D.Collision.Shapes.MassData = MassData;

   function PolygonShape() {
      PolygonShape.PolygonShape.apply(this, arguments);
      if (this.constructor === PolygonShape) this.PolygonShape.apply(this, arguments);
   };
   Box2D.Collision.Shapes.PolygonShape = PolygonShape;

   function Shape() {
      Shape.Shape.apply(this, arguments);
      if (this.constructor === Shape) this.Shape.apply(this, arguments);
   };
   Box2D.Collision.Shapes.Shape = Shape;
   Box2D.Common.internal = 'Box2D.Common.internal';

   function Color() {
      Color.Color.apply(this, arguments);
      if (this.constructor === Color) this.Color.apply(this, arguments);
   };
   Box2D.Common.Color = Color;

   function Settings() {
      Settings.Settings.apply(this, arguments);
   };
   Box2D.Common.Settings = Settings;

   function Mat22() {
      Mat22.Mat22.apply(this, arguments);
      if (this.constructor === Mat22) this.Mat22.apply(this, arguments);
   };
   Box2D.Common.Math.Mat22 = Mat22;

   function Mat33() {
      Mat33.Mat33.apply(this, arguments);
      if (this.constructor === Mat33) this.Mat33.apply(this, arguments);
   };
   Box2D.Common.Math.Mat33 = Mat33;

   function Math() {
      Math.Math.apply(this, arguments);
   };
   Box2D.Common.Math.Math = Math;

   function Sweep() {
      Sweep.Sweep.apply(this, arguments);
   };
   Box2D.Common.Math.Sweep = Sweep;

   function Transform() {
      Transform.Transform.apply(this, arguments);
      if (this.constructor === Transform) this.Transform.apply(this, arguments);
   };
   Box2D.Common.Math.Transform = Transform;

   function Vec2() {
      Vec2.Vec2.apply(this, arguments);
      if (this.constructor === Vec2) this.Vec2.apply(this, arguments);
   };
   Box2D.Common.Math.Vec2 = Vec2;

   function Vec3() {
      Vec3.Vec3.apply(this, arguments);
      if (this.constructor === Vec3) this.Vec3.apply(this, arguments);
   };
   Box2D.Common.Math.Vec3 = Vec3;

   function Body() {
      Body.Body.apply(this, arguments);
      if (this.constructor === Body) this.Body.apply(this, arguments);
   };
   Box2D.Dynamics.Body = Body;

   function BodyDef() {
      BodyDef.BodyDef.apply(this, arguments);
      if (this.constructor === BodyDef) this.BodyDef.apply(this, arguments);
   };
   Box2D.Dynamics.BodyDef = BodyDef;

   function ContactFilter() {
      ContactFilter.ContactFilter.apply(this, arguments);
   };
   Box2D.Dynamics.ContactFilter = ContactFilter;

   function ContactImpulse() {
      ContactImpulse.ContactImpulse.apply(this, arguments);
   };
   Box2D.Dynamics.ContactImpulse = ContactImpulse;

   function ContactListener() {
      ContactListener.ContactListener.apply(this, arguments);
   };
   Box2D.Dynamics.ContactListener = ContactListener;

   function ContactManager() {
      ContactManager.ContactManager.apply(this, arguments);
      if (this.constructor === ContactManager) this.ContactManager.apply(this, arguments);
   };
   Box2D.Dynamics.ContactManager = ContactManager;

   function DebugDraw() {
      DebugDraw.DebugDraw.apply(this, arguments);
      if (this.constructor === DebugDraw) this.DebugDraw.apply(this, arguments);
   };
   Box2D.Dynamics.DebugDraw = DebugDraw;

   function DestructionListener() {
      DestructionListener.DestructionListener.apply(this, arguments);
   };
   Box2D.Dynamics.DestructionListener = DestructionListener;

   function FilterData() {
      FilterData.FilterData.apply(this, arguments);
   };
   Box2D.Dynamics.FilterData = FilterData;

   function Fixture() {
      Fixture.Fixture.apply(this, arguments);
      if (this.constructor === Fixture) this.Fixture.apply(this, arguments);
   };
   Box2D.Dynamics.Fixture = Fixture;

   function FixtureDef() {
      FixtureDef.FixtureDef.apply(this, arguments);
      if (this.constructor === FixtureDef) this.FixtureDef.apply(this, arguments);
   };
   Box2D.Dynamics.FixtureDef = FixtureDef;

   function Island() {
      Island.Island.apply(this, arguments);
      if (this.constructor === Island) this.Island.apply(this, arguments);
   };
   Box2D.Dynamics.Island = Island;

   function TimeStep() {
      TimeStep.TimeStep.apply(this, arguments);
   };
   Box2D.Dynamics.TimeStep = TimeStep;

   function World() {
      World.World.apply(this, arguments);
      if (this.constructor === World) this.World.apply(this, arguments);
   };
   Box2D.Dynamics.World = World;

   function CircleContact() {
      CircleContact.CircleContact.apply(this, arguments);
   };
   Box2D.Dynamics.Contacts.CircleContact = CircleContact;

   function Contact() {
      Contact.Contact.apply(this, arguments);
      if (this.constructor === Contact) this.Contact.apply(this, arguments);
   };
   Box2D.Dynamics.Contacts.Contact = Contact;

   function ContactConstraint() {
      ContactConstraint.ContactConstraint.apply(this, arguments);
      if (this.constructor === ContactConstraint) this.ContactConstraint.apply(this, arguments);
   };
   Box2D.Dynamics.Contacts.ContactConstraint = ContactConstraint;

   function ContactConstraintPoint() {
      ContactConstraintPoint.ContactConstraintPoint.apply(this, arguments);
   };
   Box2D.Dynamics.Contacts.ContactConstraintPoint = ContactConstraintPoint;

   function ContactEdge() {
      ContactEdge.ContactEdge.apply(this, arguments);
   };
   Box2D.Dynamics.Contacts.ContactEdge = ContactEdge;

   function ContactFactory() {
      ContactFactory.ContactFactory.apply(this, arguments);
      if (this.constructor === ContactFactory) this.ContactFactory.apply(this, arguments);
   };
   Box2D.Dynamics.Contacts.ContactFactory = ContactFactory;

   function ContactRegister() {
      ContactRegister.ContactRegister.apply(this, arguments);
   };
   Box2D.Dynamics.Contacts.ContactRegister = ContactRegister;

   function ContactResult() {
      ContactResult.ContactResult.apply(this, arguments);
   };
   Box2D.Dynamics.Contacts.ContactResult = ContactResult;

   function ContactSolver() {
      ContactSolver.ContactSolver.apply(this, arguments);
      if (this.constructor === ContactSolver) this.ContactSolver.apply(this, arguments);
   };
   Box2D.Dynamics.Contacts.ContactSolver = ContactSolver;

   function EdgeAndCircleContact() {
      EdgeAndCircleContact.EdgeAndCircleContact.apply(this, arguments);
   };
   Box2D.Dynamics.Contacts.EdgeAndCircleContact = EdgeAndCircleContact;

   function NullContact() {
      NullContact.NullContact.apply(this, arguments);
      if (this.constructor === NullContact) this.NullContact.apply(this, arguments);
   };
   Box2D.Dynamics.Contacts.NullContact = NullContact;

   function PolyAndCircleContact() {
      PolyAndCircleContact.PolyAndCircleContact.apply(this, arguments);
   };
   Box2D.Dynamics.Contacts.PolyAndCircleContact = PolyAndCircleContact;

   function PolyAndEdgeContact() {
      PolyAndEdgeContact.PolyAndEdgeContact.apply(this, arguments);
   };
   Box2D.Dynamics.Contacts.PolyAndEdgeContact = PolyAndEdgeContact;

   function PolygonContact() {
      PolygonContact.PolygonContact.apply(this, arguments);
   };
   Box2D.Dynamics.Contacts.PolygonContact = PolygonContact;

   function PositionSolverManifold() {
      PositionSolverManifold.PositionSolverManifold.apply(this, arguments);
      if (this.constructor === PositionSolverManifold) this.PositionSolverManifold.apply(this, arguments);
   };
   Box2D.Dynamics.Contacts.PositionSolverManifold = PositionSolverManifold;

   function BuoyancyController() {
      BuoyancyController.BuoyancyController.apply(this, arguments);
   };
   Box2D.Dynamics.Controllers.BuoyancyController = BuoyancyController;

   function ConstantAccelController() {
      ConstantAccelController.ConstantAccelController.apply(this, arguments);
   };
   Box2D.Dynamics.Controllers.ConstantAccelController = ConstantAccelController;

   function ConstantForceController() {
      ConstantForceController.ConstantForceController.apply(this, arguments);
   };
   Box2D.Dynamics.Controllers.ConstantForceController = ConstantForceController;

   function Controller() {
      Controller.Controller.apply(this, arguments);
   };
   Box2D.Dynamics.Controllers.Controller = Controller;

   function ControllerEdge() {
      ControllerEdge.ControllerEdge.apply(this, arguments);
   };
   Box2D.Dynamics.Controllers.ControllerEdge = ControllerEdge;

   function GravityController() {
      GravityController.GravityController.apply(this, arguments);
   };
   Box2D.Dynamics.Controllers.GravityController = GravityController;

   function TensorDampingController() {
      TensorDampingController.TensorDampingController.apply(this, arguments);
   };
   Box2D.Dynamics.Controllers.TensorDampingController = TensorDampingController;

   function DistanceJoint() {
      DistanceJoint.DistanceJoint.apply(this, arguments);
      if (this.constructor === DistanceJoint) this.DistanceJoint.apply(this, arguments);
   };
   Box2D.Dynamics.Joints.DistanceJoint = DistanceJoint;

   function DistanceJointDef() {
      DistanceJointDef.DistanceJointDef.apply(this, arguments);
      if (this.constructor === DistanceJointDef) this.DistanceJointDef.apply(this, arguments);
   };
   Box2D.Dynamics.Joints.DistanceJointDef = DistanceJointDef;

   function FrictionJoint() {
      FrictionJoint.FrictionJoint.apply(this, arguments);
      if (this.constructor === FrictionJoint) this.FrictionJoint.apply(this, arguments);
   };
   Box2D.Dynamics.Joints.FrictionJoint = FrictionJoint;

   function FrictionJointDef() {
      FrictionJointDef.FrictionJointDef.apply(this, arguments);
      if (this.constructor === FrictionJointDef) this.FrictionJointDef.apply(this, arguments);
   };
   Box2D.Dynamics.Joints.FrictionJointDef = FrictionJointDef;

   function GearJoint() {
      GearJoint.GearJoint.apply(this, arguments);
      if (this.constructor === GearJoint) this.GearJoint.apply(this, arguments);
   };
   Box2D.Dynamics.Joints.GearJoint = GearJoint;

   function GearJointDef() {
      GearJointDef.GearJointDef.apply(this, arguments);
      if (this.constructor === GearJointDef) this.GearJointDef.apply(this, arguments);
   };
   Box2D.Dynamics.Joints.GearJointDef = GearJointDef;

   function Jacobian() {
      Jacobian.Jacobian.apply(this, arguments);
   };
   Box2D.Dynamics.Joints.Jacobian = Jacobian;

   function Joint() {
      Joint.Joint.apply(this, arguments);
      if (this.constructor === Joint) this.Joint.apply(this, arguments);
   };
   Box2D.Dynamics.Joints.Joint = Joint;

   function JointDef() {
      JointDef.JointDef.apply(this, arguments);
      if (this.constructor === JointDef) this.JointDef.apply(this, arguments);
   };
   Box2D.Dynamics.Joints.JointDef = JointDef;

   function JointEdge() {
      JointEdge.JointEdge.apply(this, arguments);
   };
   Box2D.Dynamics.Joints.JointEdge = JointEdge;

   function LineJoint() {
      LineJoint.LineJoint.apply(this, arguments);
      if (this.constructor === LineJoint) this.LineJoint.apply(this, arguments);
   };
   Box2D.Dynamics.Joints.LineJoint = LineJoint;

   function LineJointDef() {
      LineJointDef.LineJointDef.apply(this, arguments);
      if (this.constructor === LineJointDef) this.LineJointDef.apply(this, arguments);
   };
   Box2D.Dynamics.Joints.LineJointDef = LineJointDef;

   function MouseJoint() {
      MouseJoint.MouseJoint.apply(this, arguments);
      if (this.constructor === MouseJoint) this.MouseJoint.apply(this, arguments);
   };
   Box2D.Dynamics.Joints.MouseJoint = MouseJoint;

   function MouseJointDef() {
      MouseJointDef.MouseJointDef.apply(this, arguments);
      if (this.constructor === MouseJointDef) this.MouseJointDef.apply(this, arguments);
   };
   Box2D.Dynamics.Joints.MouseJointDef = MouseJointDef;

   function PrismaticJoint() {
      PrismaticJoint.PrismaticJoint.apply(this, arguments);
      if (this.constructor === PrismaticJoint) this.PrismaticJoint.apply(this, arguments);
   };
   Box2D.Dynamics.Joints.PrismaticJoint = PrismaticJoint;

   function PrismaticJointDef() {
      PrismaticJointDef.PrismaticJointDef.apply(this, arguments);
      if (this.constructor === PrismaticJointDef) this.PrismaticJointDef.apply(this, arguments);
   };
   Box2D.Dynamics.Joints.PrismaticJointDef = PrismaticJointDef;

   function PulleyJoint() {
      PulleyJoint.PulleyJoint.apply(this, arguments);
      if (this.constructor === PulleyJoint) this.PulleyJoint.apply(this, arguments);
   };
   Box2D.Dynamics.Joints.PulleyJoint = PulleyJoint;

   function PulleyJointDef() {
      PulleyJointDef.PulleyJointDef.apply(this, arguments);
      if (this.constructor === PulleyJointDef) this.PulleyJointDef.apply(this, arguments);
   };
   Box2D.Dynamics.Joints.PulleyJointDef = PulleyJointDef;

   function RevoluteJoint() {
      RevoluteJoint.RevoluteJoint.apply(this, arguments);
      if (this.constructor === RevoluteJoint) this.RevoluteJoint.apply(this, arguments);
   };
   Box2D.Dynamics.Joints.RevoluteJoint = RevoluteJoint;

   function RevoluteJointDef() {
      RevoluteJointDef.RevoluteJointDef.apply(this, arguments);
      if (this.constructor === RevoluteJointDef) this.RevoluteJointDef.apply(this, arguments);
   };
   Box2D.Dynamics.Joints.RevoluteJointDef = RevoluteJointDef;

   function WeldJoint() {
      WeldJoint.WeldJoint.apply(this, arguments);
      if (this.constructor === WeldJoint) this.WeldJoint.apply(this, arguments);
   };
   Box2D.Dynamics.Joints.WeldJoint = WeldJoint;

   function WeldJointDef() {
      WeldJointDef.WeldJointDef.apply(this, arguments);
      if (this.constructor === WeldJointDef) this.WeldJointDef.apply(this, arguments);
   };
   Box2D.Dynamics.Joints.WeldJointDef = WeldJointDef;
})(); //definitions
Box2D.postDefs = [];
(function () {
   var CircleShape = Box2D.Collision.Shapes.CircleShape,
      EdgeChainDef = Box2D.Collision.Shapes.EdgeChainDef,
      EdgeShape = Box2D.Collision.Shapes.EdgeShape,
      MassData = Box2D.Collision.Shapes.MassData,
      PolygonShape = Box2D.Collision.Shapes.PolygonShape,
      Shape = Box2D.Collision.Shapes.Shape,
      Color = Box2D.Common.Color,
      internal = Box2D.Common.internal,
      Settings = Box2D.Common.Settings,
      Mat22 = Box2D.Common.Math.Mat22,
      Mat33 = Box2D.Common.Math.Mat33,
      Math = Box2D.Common.Math.Math,
      Sweep = Box2D.Common.Math.Sweep,
      Transform = Box2D.Common.Math.Transform,
      Vec2 = Box2D.Common.Math.Vec2,
      Vec3 = Box2D.Common.Math.Vec3,
      AABB = Box2D.Collision.AABB,
      Bound = Box2D.Collision.Bound,
      BoundValues = Box2D.Collision.BoundValues,
      Collision = Box2D.Collision.Collision,
      ContactID = Box2D.Collision.ContactID,
      ContactPoint = Box2D.Collision.ContactPoint,
      Distance = Box2D.Collision.Distance,
      DistanceInput = Box2D.Collision.DistanceInput,
      DistanceOutput = Box2D.Collision.DistanceOutput,
      DistanceProxy = Box2D.Collision.DistanceProxy,
      DynamicTree = Box2D.Collision.DynamicTree,
      DynamicTreeBroadPhase = Box2D.Collision.DynamicTreeBroadPhase,
      DynamicTreeNode = Box2D.Collision.DynamicTreeNode,
      DynamicTreePair = Box2D.Collision.DynamicTreePair,
      Manifold = Box2D.Collision.Manifold,
      ManifoldPoint = Box2D.Collision.ManifoldPoint,
      Point = Box2D.Collision.Point,
      RayCastInput = Box2D.Collision.RayCastInput,
      RayCastOutput = Box2D.Collision.RayCastOutput,
      Segment = Box2D.Collision.Segment,
      SeparationFunction = Box2D.Collision.SeparationFunction,
      Simplex = Box2D.Collision.Simplex,
      SimplexCache = Box2D.Collision.SimplexCache,
      SimplexVertex = Box2D.Collision.SimplexVertex,
      TimeOfImpact = Box2D.Collision.TimeOfImpact,
      TOIInput = Box2D.Collision.TOIInput,
      WorldManifold = Box2D.Collision.WorldManifold,
      ClipVertex = Box2D.Collision.ClipVertex,
      Features = Box2D.Collision.Features,
      IBroadPhase = Box2D.Collision.IBroadPhase;

   AABB.AABB = function () {
      this.lowerBound = new Vec2();
      this.upperBound = new Vec2();
   };
   AABB.prototype.IsValid = function () {
      var dX = this.upperBound.x - this.lowerBound.x;
      var dY = this.upperBound.y - this.lowerBound.y;
      var valid = dX >= 0.0 && dY >= 0.0;
      valid = valid && this.lowerBound.IsValid() && this.upperBound.IsValid();
      return valid;
   }
   AABB.prototype.GetCenter = function () {
      return new Vec2((this.lowerBound.x + this.upperBound.x) / 2, (this.lowerBound.y + this.upperBound.y) / 2);
   }
   AABB.prototype.GetExtents = function () {
      return new Vec2((this.upperBound.x - this.lowerBound.x) / 2, (this.upperBound.y - this.lowerBound.y) / 2);
   }
   AABB.prototype.Contains = function (aabb) {
      var result = true;
      result = result && this.lowerBound.x <= aabb.lowerBound.x;
      result = result && this.lowerBound.y <= aabb.lowerBound.y;
      result = result && aabb.upperBound.x <= this.upperBound.x;
      result = result && aabb.upperBound.y <= this.upperBound.y;
      return result;
   }
   AABB.prototype.RayCast = function (output, input) {
      var tmin = (-Number.MAX_VALUE);
      var tmax = Number.MAX_VALUE;
      var pX = input.p1.x;
      var pY = input.p1.y;
      var dX = input.p2.x - input.p1.x;
      var dY = input.p2.y - input.p1.y;
      var absDX = Math.abs(dX);
      var absDY = Math.abs(dY);
      var normal = output.normal;
      var inv_d = 0;
      var t1 = 0;
      var t2 = 0;
      var t3 = 0;
      var s = 0; {
         if (absDX < Number.MIN_VALUE) {
            if (pX < this.lowerBound.x || this.upperBound.x < pX) return false;
         }
         else {
            inv_d = 1.0 / dX;
            t1 = (this.lowerBound.x - pX) * inv_d;
            t2 = (this.upperBound.x - pX) * inv_d;
            s = (-1.0);
            if (t1 > t2) {
               t3 = t1;
               t1 = t2;
               t2 = t3;
               s = 1.0;
            }
            if (t1 > tmin) {
               normal.x = s;
               normal.y = 0;
               tmin = t1;
            }
            tmax = Math.min(tmax, t2);
            if (tmin > tmax) return false;
         }
      } {
         if (absDY < Number.MIN_VALUE) {
            if (pY < this.lowerBound.y || this.upperBound.y < pY) return false;
         }
         else {
            inv_d = 1.0 / dY;
            t1 = (this.lowerBound.y - pY) * inv_d;
            t2 = (this.upperBound.y - pY) * inv_d;
            s = (-1.0);
            if (t1 > t2) {
               t3 = t1;
               t1 = t2;
               t2 = t3;
               s = 1.0;
            }
            if (t1 > tmin) {
               normal.y = s;
               normal.x = 0;
               tmin = t1;
            }
            tmax = Math.min(tmax, t2);
            if (tmin > tmax) return false;
         }
      }
      output.fraction = tmin;
      return true;
   }
   AABB.prototype.TestOverlap = function (other) {
      var d1X = other.lowerBound.x - this.upperBound.x;
      var d1Y = other.lowerBound.y - this.upperBound.y;
      var d2X = this.lowerBound.x - other.upperBound.x;
      var d2Y = this.lowerBound.y - other.upperBound.y;
      if (d1X > 0.0 || d1Y > 0.0) return false;
      if (d2X > 0.0 || d2Y > 0.0) return false;
      return true;
   }
   AABB.Combine = function (aabb1, aab) {
      var aabb = new AABB();
      aabb.Combine(aabb1, aab);
      return aabb;
   }
   AABB.prototype.Combine = function (aabb1, aab) {
      this.lowerBound.x = Math.min(aabb1.lowerBound.x, aab.lowerBound.x);
      this.lowerBound.y = Math.min(aabb1.lowerBound.y, aab.lowerBound.y);
      this.upperBound.x = Math.max(aabb1.upperBound.x, aab.upperBound.x);
      this.upperBound.y = Math.max(aabb1.upperBound.y, aab.upperBound.y);
   }
   Bound.Bound = function () {};
   Bound.prototype.IsLower = function () {
      return (this.value & 1) == 0;
   }
   Bound.prototype.IsUpper = function () {
      return (this.value & 1) == 1;
   }
   Bound.prototype.Swap = function (b) {
      var tempValue = this.value;
      var tempProxy = this.proxy;
      var tempStabbingCount = this.stabbingCount;
      this.value = b.value;
      this.proxy = b.proxy;
      this.stabbingCount = b.stabbingCount;
      b.value = tempValue;
      b.proxy = tempProxy;
      b.stabbingCount = tempStabbingCount;
   }
   BoundValues.BoundValues = function () {};
   BoundValues.prototype.BoundValues = function () {
      this.lowerValues = new Vector_a2j_Number();
      this.lowerValues[0] = 0.0;
      this.lowerValues[1] = 0.0;
      this.upperValues = new Vector_a2j_Number();
      this.upperValues[0] = 0.0;
      this.upperValues[1] = 0.0;
   }
   Collision.Collision = function () {};
   Collision.ClipSegmentToLine = function (vOut, vIn, normal, offset) {
      if (offset === undefined) offset = 0;
      var cv;
      var numOut = 0;
      cv = vIn[0];
      var vIn0 = cv.v;
      cv = vIn[1];
      var vIn1 = cv.v;
      var distance0 = normal.x * vIn0.x + normal.y * vIn0.y - offset;
      var distance1 = normal.x * vIn1.x + normal.y * vIn1.y - offset;
      if (distance0 <= 0.0) vOut[numOut++].Set(vIn[0]);
      if (distance1 <= 0.0) vOut[numOut++].Set(vIn[1]);
      if (distance0 * distance1 < 0.0) {
         var interp = distance0 / (distance0 - distance1);
         cv = vOut[numOut];
         var tVec = cv.v;
         tVec.x = vIn0.x + interp * (vIn1.x - vIn0.x);
         tVec.y = vIn0.y + interp * (vIn1.y - vIn0.y);
         cv = vOut[numOut];
         var cv2;
         if (distance0 > 0.0) {
            cv2 = vIn[0];
            cv.id = cv2.id;
         }
         else {
            cv2 = vIn[1];
            cv.id = cv2.id;
         }++numOut;
      }
      return numOut;
   }
   Collision.EdgeSeparation = function (poly1, xf1, edge1, poly2, xf2) {
      if (edge1 === undefined) edge1 = 0;
      var count1 = parseInt(poly1.vertexCount);
      var vertices1 = poly1.vertices;
      var normals1 = poly1.normals;
      var count2 = parseInt(poly2.vertexCount);
      var vertices2 = poly2.vertices;
      var tMat;
      var tVec;
      tMat = xf1.R;
      tVec = normals1[edge1];
      var normal1WorldX = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      var normal1WorldY = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tMat = xf2.R;
      var normal1X = (tMat.col1.x * normal1WorldX + tMat.col1.y * normal1WorldY);
      var normal1Y = (tMat.col2.x * normal1WorldX + tMat.col2.y * normal1WorldY);
      var index = 0;
      var minDot = Number.MAX_VALUE;
      for (var i = 0; i < count2; ++i) {
         tVec = vertices2[i];
         var dot = tVec.x * normal1X + tVec.y * normal1Y;
         if (dot < minDot) {
            minDot = dot;
            index = i;
         }
      }
      tVec = vertices1[edge1];
      tMat = xf1.R;
      var v1X = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      var v1Y = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tVec = vertices2[index];
      tMat = xf2.R;
      var v2X = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      var v2Y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      v2X -= v1X;
      v2Y -= v1Y;
      var separation = v2X * normal1WorldX + v2Y * normal1WorldY;
      return separation;
   }
   Collision.FindMaxSeparation = function (edgeIndex, poly1, xf1, poly2, xf2) {
      var count1 = parseInt(poly1.vertexCount);
      var normals1 = poly1.normals;
      var tVec;
      var tMat;
      tMat = xf2.R;
      tVec = poly2.centroid;
      var dX = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      var dY = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tMat = xf1.R;
      tVec = poly1.centroid;
      dX -= xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      dY -= xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      var dLocal1X = (dX * xf1.R.col1.x + dY * xf1.R.col1.y);
      var dLocal1Y = (dX * xf1.R.col2.x + dY * xf1.R.col2.y);
      var edge = 0;
      var maxDot = (-Number.MAX_VALUE);
      for (var i = 0; i < count1; ++i) {
         tVec = normals1[i];
         var dot = (tVec.x * dLocal1X + tVec.y * dLocal1Y);
         if (dot > maxDot) {
            maxDot = dot;
            edge = i;
         }
      }
      var s = Collision.EdgeSeparation(poly1, xf1, edge, poly2, xf2);
      var prevEdge = parseInt(edge - 1 >= 0 ? edge - 1 : count1 - 1);
      var sPrev = Collision.EdgeSeparation(poly1, xf1, prevEdge, poly2, xf2);
      var nextEdge = parseInt(edge + 1 < count1 ? edge + 1 : 0);
      var sNext = Collision.EdgeSeparation(poly1, xf1, nextEdge, poly2, xf2);
      var bestEdge = 0;
      var bestSeparation = 0;
      var increment = 0;
      if (sPrev > s && sPrev > sNext) {
         increment = (-1);
         bestEdge = prevEdge;
         bestSeparation = sPrev;
      }
      else if (sNext > s) {
         increment = 1;
         bestEdge = nextEdge;
         bestSeparation = sNext;
      }
      else {
         edgeIndex[0] = edge;
         return s;
      }
      while (true) {
         if (increment == (-1)) edge = bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1;
         else edge = bestEdge + 1 < count1 ? bestEdge + 1 : 0;s = Collision.EdgeSeparation(poly1, xf1, edge, poly2, xf2);
         if (s > bestSeparation) {
            bestEdge = edge;
            bestSeparation = s;
         }
         else {
            break;
         }
      }
      edgeIndex[0] = bestEdge;
      return bestSeparation;
   }
   Collision.FindIncidentEdge = function (c, poly1, xf1, edge1, poly2, xf2) {
      if (edge1 === undefined) edge1 = 0;
      var count1 = parseInt(poly1.vertexCount);
      var normals1 = poly1.normals;
      var count2 = parseInt(poly2.vertexCount);
      var vertices2 = poly2.vertices;
      var normals2 = poly2.normals;
      var tMat;
      var tVec;
      tMat = xf1.R;
      tVec = normals1[edge1];
      var normal1X = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      var normal1Y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tMat = xf2.R;
      var tX = (tMat.col1.x * normal1X + tMat.col1.y * normal1Y);
      normal1Y = (tMat.col2.x * normal1X + tMat.col2.y * normal1Y);
      normal1X = tX;
      var index = 0;
      var minDot = Number.MAX_VALUE;
      for (var i = 0; i < count2; ++i) {
         tVec = normals2[i];
         var dot = (normal1X * tVec.x + normal1Y * tVec.y);
         if (dot < minDot) {
            minDot = dot;
            index = i;
         }
      }
      var tClip;
      var i1 = parseInt(index);
      var i2 = parseInt(i1 + 1 < count2 ? i1 + 1 : 0);
      tClip = c[0];
      tVec = vertices2[i1];
      tMat = xf2.R;
      tClip.v.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      tClip.v.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tClip.id.features.referenceEdge = edge1;
      tClip.id.features.incidentEdge = i1;
      tClip.id.features.incidentVertex = 0;
      tClip = c[1];
      tVec = vertices2[i2];
      tMat = xf2.R;
      tClip.v.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      tClip.v.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tClip.id.features.referenceEdge = edge1;
      tClip.id.features.incidentEdge = i2;
      tClip.id.features.incidentVertex = 1;
   }
   Collision.MakeClipPointVector = function () {
      var r = new Vector(2);
      r[0] = new ClipVertex();
      r[1] = new ClipVertex();
      return r;
   }
   Collision.CollidePolygons = function (manifold, polyA, xfA, polyB, xfB) {
      var cv;
      manifold.pointCount = 0;
      var totalRadius = polyA.radius + polyB.radius;
      var edgeA = 0;
      Collision.s_edgeAO[0] = edgeA;
      var separationA = Collision.FindMaxSeparation(Collision.s_edgeAO, polyA, xfA, polyB, xfB);
      edgeA = Collision.s_edgeAO[0];
      if (separationA > totalRadius) return;
      var edgeB = 0;
      Collision.s_edgeBO[0] = edgeB;
      var separationB = Collision.FindMaxSeparation(Collision.s_edgeBO, polyB, xfB, polyA, xfA);
      edgeB = Collision.s_edgeBO[0];
      if (separationB > totalRadius) return;
      var poly1;
      var poly2;
      var xf1;
      var xf2;
      var edge1 = 0;
      var flip = 0;
      var k_relativeTol = 0.98;
      var k_absoluteTol = 0.001;
      var tMat;
      if (separationB > k_relativeTol * separationA + k_absoluteTol) {
         poly1 = polyB;
         poly2 = polyA;
         xf1 = xfB;
         xf2 = xfA;
         edge1 = edgeB;
         manifold.type = Manifold.e_faceB;
         flip = 1;
      }
      else {
         poly1 = polyA;
         poly2 = polyB;
         xf1 = xfA;
         xf2 = xfB;
         edge1 = edgeA;
         manifold.type = Manifold.e_faceA;
         flip = 0;
      }
      var incidentEdge = Collision.s_incidentEdge;
      Collision.FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);
      var count1 = parseInt(poly1.vertexCount);
      var vertices1 = poly1.vertices;
      var local_v11 = vertices1[edge1];
      var local_v12;
      if (edge1 + 1 < count1) {
         local_v12 = vertices1[parseInt(edge1 + 1)];
      }
      else {
         local_v12 = vertices1[0];
      }
      var localTangent = Collision.s_localTangent;
      localTangent.Set(local_v12.x - local_v11.x, local_v12.y - local_v11.y);
      localTangent.Normalize();
      var localNormal = Collision.s_localNormal;
      localNormal.x = localTangent.y;
      localNormal.y = (-localTangent.x);
      var planePoint = Collision.s_planePoint;
      planePoint.Set(0.5 * (local_v11.x + local_v12.x), 0.5 * (local_v11.y + local_v12.y));
      var tangent = Collision.s_tangent;
      tMat = xf1.R;
      tangent.x = (tMat.col1.x * localTangent.x + tMat.col2.x * localTangent.y);
      tangent.y = (tMat.col1.y * localTangent.x + tMat.col2.y * localTangent.y);
      var tangent2 = Collision.s_tangent2;
      tangent2.x = (-tangent.x);
      tangent2.y = (-tangent.y);
      var normal = Collision.s_normal;
      normal.x = tangent.y;
      normal.y = (-tangent.x);
      var v11 = Collision.s_v11;
      var v12 = Collision.s_v12;
      v11.x = xf1.position.x + (tMat.col1.x * local_v11.x + tMat.col2.x * local_v11.y);
      v11.y = xf1.position.y + (tMat.col1.y * local_v11.x + tMat.col2.y * local_v11.y);
      v12.x = xf1.position.x + (tMat.col1.x * local_v12.x + tMat.col2.x * local_v12.y);
      v12.y = xf1.position.y + (tMat.col1.y * local_v12.x + tMat.col2.y * local_v12.y);
      var frontOffset = normal.x * v11.x + normal.y * v11.y;
      var sideOffset1 = (-tangent.x * v11.x) - tangent.y * v11.y + totalRadius;
      var sideOffset2 = tangent.x * v12.x + tangent.y * v12.y + totalRadius;
      var clipPoints1 = Collision.s_clipPoints1;
      var clipPoints2 = Collision.s_clipPoints2;
      var np = 0;
      np = Collision.ClipSegmentToLine(clipPoints1, incidentEdge, tangent2, sideOffset1);
      if (np < 2) return;
      np = Collision.ClipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2);
      if (np < 2) return;
      manifold.localPlaneNormal.SetV(localNormal);
      manifold.localPoint.SetV(planePoint);
      var pointCount = 0;
      for (var i = 0; i < Settings._maxManifoldPoints; ++i) {
         cv = clipPoints2[i];
         var separation = normal.x * cv.v.x + normal.y * cv.v.y - frontOffset;
         if (separation <= totalRadius) {
            var cp = manifold.points[pointCount];
            tMat = xf2.R;
            var tX = cv.v.x - xf2.position.x;
            var tY = cv.v.y - xf2.position.y;
            cp.localPoint.x = (tX * tMat.col1.x + tY * tMat.col1.y);
            cp.localPoint.y = (tX * tMat.col2.x + tY * tMat.col2.y);
            cp.id.Set(cv.id);
            cp.id.features.flip = flip;
            ++pointCount;
         }
      }
      manifold.pointCount = pointCount;
   }
   Collision.CollideCircles = function (manifold, circle1, xf1, circle2, xf2) {
      manifold.pointCount = 0;
      var tMat;
      var tVec;
      tMat = xf1.R;
      tVec = circle1.p;
      var p1X = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      var p1Y = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tMat = xf2.R;
      tVec = circle2.p;
      var p2X = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      var p2Y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      var dX = p2X - p1X;
      var dY = p2Y - p1Y;
      var distSqr = dX * dX + dY * dY;
      var radius = circle1.radius + circle2.radius;
      if (distSqr > radius * radius) {
         return;
      }
      manifold.type = Manifold.e_circles;
      manifold.localPoint.SetV(circle1.p);
      manifold.localPlaneNormal.SetZero();
      manifold.pointCount = 1;
      manifold.points[0].localPoint.SetV(circle2.p);
      manifold.points[0].id.key = 0;
   }
   Collision.CollidePolygonAndCircle = function (manifold, polygon, xf1, circle, xf2) {
      manifold.pointCount = 0;
      var tPoint;
      var dX = 0;
      var dY = 0;
      var positionX = 0;
      var positionY = 0;
      var tVec;
      var tMat;
      tMat = xf2.R;
      tVec = circle.p;
      var cX = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      var cY = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      dX = cX - xf1.position.x;
      dY = cY - xf1.position.y;
      tMat = xf1.R;
      var cLocalX = (dX * tMat.col1.x + dY * tMat.col1.y);
      var cLocalY = (dX * tMat.col2.x + dY * tMat.col2.y);
      var dist = 0;
      var normalIndex = 0;
      var separation = (-Number.MAX_VALUE);
      var radius = polygon.radius + circle.radius;
      var vertexCount = parseInt(polygon.vertexCount);
      var vertices = polygon.vertices;
      var normals = polygon.normals;
      for (var i = 0; i < vertexCount; ++i) {
         tVec = vertices[i];
         dX = cLocalX - tVec.x;
         dY = cLocalY - tVec.y;
         tVec = normals[i];
         var s = tVec.x * dX + tVec.y * dY;
         if (s > radius) {
            return;
         }
         if (s > separation) {
            separation = s;
            normalIndex = i;
         }
      }
      var vertIndex1 = parseInt(normalIndex);
      var vertIndex2 = parseInt(vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0);
      var v1 = vertices[vertIndex1];
      var v2 = vertices[vertIndex2];
      if (separation < Number.MIN_VALUE) {
         manifold.pointCount = 1;
         manifold.type = Manifold.e_faceA;
         manifold.localPlaneNormal.SetV(normals[normalIndex]);
         manifold.localPoint.x = 0.5 * (v1.x + v2.x);
         manifold.localPoint.y = 0.5 * (v1.y + v2.y);
         manifold.points[0].localPoint.SetV(circle.p);
         manifold.points[0].id.key = 0;
         return;
      }
      var u1 = (cLocalX - v1.x) * (v2.x - v1.x) + (cLocalY - v1.y) * (v2.y - v1.y);
      var u2 = (cLocalX - v2.x) * (v1.x - v2.x) + (cLocalY - v2.y) * (v1.y - v2.y);
      if (u1 <= 0.0) {
         if ((cLocalX - v1.x) * (cLocalX - v1.x) + (cLocalY - v1.y) * (cLocalY - v1.y) > radius * radius) return;
         manifold.pointCount = 1;
         manifold.type = Manifold.e_faceA;
         manifold.localPlaneNormal.x = cLocalX - v1.x;
         manifold.localPlaneNormal.y = cLocalY - v1.y;
         manifold.localPlaneNormal.Normalize();
         manifold.localPoint.SetV(v1);
         manifold.points[0].localPoint.SetV(circle.p);
         manifold.points[0].id.key = 0;
      }
      else if (u2 <= 0) {
         if ((cLocalX - v2.x) * (cLocalX - v2.x) + (cLocalY - v2.y) * (cLocalY - v2.y) > radius * radius) return;
         manifold.pointCount = 1;
         manifold.type = Manifold.e_faceA;
         manifold.localPlaneNormal.x = cLocalX - v2.x;
         manifold.localPlaneNormal.y = cLocalY - v2.y;
         manifold.localPlaneNormal.Normalize();
         manifold.localPoint.SetV(v2);
         manifold.points[0].localPoint.SetV(circle.p);
         manifold.points[0].id.key = 0;
      }
      else {
         var faceCenterX = 0.5 * (v1.x + v2.x);
         var faceCenterY = 0.5 * (v1.y + v2.y);
         separation = (cLocalX - faceCenterX) * normals[vertIndex1].x + (cLocalY - faceCenterY) * normals[vertIndex1].y;
         if (separation > radius) return;
         manifold.pointCount = 1;
         manifold.type = Manifold.e_faceA;
         manifold.localPlaneNormal.x = normals[vertIndex1].x;
         manifold.localPlaneNormal.y = normals[vertIndex1].y;
         manifold.localPlaneNormal.Normalize();
         manifold.localPoint.Set(faceCenterX, faceCenterY);
         manifold.points[0].localPoint.SetV(circle.p);
         manifold.points[0].id.key = 0;
      }
   }
   Collision.TestOverlap = function (a, b) {
      var t1 = b.lowerBound;
      var t2 = a.upperBound;
      var d1X = t1.x - t2.x;
      var d1Y = t1.y - t2.y;
      t1 = a.lowerBound;
      t2 = b.upperBound;
      var d2X = t1.x - t2.x;
      var d2Y = t1.y - t2.y;
      if (d1X > 0.0 || d1Y > 0.0) return false;
      if (d2X > 0.0 || d2Y > 0.0) return false;
      return true;
   }
   Box2D.postDefs.push(function () {
      Box2D.Collision.Collision.s_incidentEdge = Collision.MakeClipPointVector();
      Box2D.Collision.Collision.s_clipPoints1 = Collision.MakeClipPointVector();
      Box2D.Collision.Collision.s_clipPoints2 = Collision.MakeClipPointVector();
      Box2D.Collision.Collision.s_edgeAO = new Vector_a2j_Number(1);
      Box2D.Collision.Collision.s_edgeBO = new Vector_a2j_Number(1);
      Box2D.Collision.Collision.s_localTangent = new Vec2();
      Box2D.Collision.Collision.s_localNormal = new Vec2();
      Box2D.Collision.Collision.s_planePoint = new Vec2();
      Box2D.Collision.Collision.s_normal = new Vec2();
      Box2D.Collision.Collision.s_tangent = new Vec2();
      Box2D.Collision.Collision.s_tangent2 = new Vec2();
      Box2D.Collision.Collision.s_v11 = new Vec2();
      Box2D.Collision.Collision.s_v12 = new Vec2();
      Box2D.Collision.Collision.CollidePolyTempVec = new Vec2();
      Box2D.Collision.Collision._nullFeature = 0x000000ff;
   });
   ContactID.ContactID = function () {
      this.features = new Features();
   };
   ContactID.prototype.ContactID = function () {
      this.features._id = this;
   }
   ContactID.prototype.Set = function (id) {
      this.key = id._key;
   }
   ContactID.prototype.Copy = function () {
      var id = new ContactID();
      id.key = this.key;
      return id;
   }
   Object.defineProperty(ContactID.prototype, 'key', {
      enumerable: false,
      configurable: true,
      get: function () {
         return this._key;
      }
   });
   Object.defineProperty(ContactID.prototype, 'key', {
      enumerable: false,
      configurable: true,
      set: function (value) {
         if (value === undefined) value = 0;
         this._key = value;
         this.features._referenceEdge = this._key & 0x000000ff;
         this.features._incidentEdge = ((this._key & 0x0000ff00) >> 8) & 0x000000ff;
         this.features._incidentVertex = ((this._key & 0x00ff0000) >> 16) & 0x000000ff;
         this.features._flip = ((this._key & 0xff000000) >> 24) & 0x000000ff;
      }
   });
   ContactPoint.ContactPoint = function () {
      this.position = new Vec2();
      this.velocity = new Vec2();
      this.normal = new Vec2();
      this.id = new ContactID();
   };
   Distance.Distance = function () {};
   Distance.Distance = function (output, cache, input) {
      ++Distance._gjkCalls;
      var proxyA = input.proxyA;
      var proxyB = input.proxyB;
      var transformA = input.transformA;
      var transformB = input.transformB;
      var simplex = Distance.s_simplex;
      simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB);
      var vertices = simplex.vertices;
      var k_maxIters = 20;
      var saveA = Distance.s_saveA;
      var saveB = Distance.s_saveB;
      var saveCount = 0;
      var closestPoint = simplex.GetClosestPoint();
      var distanceSqr1 = closestPoint.LengthSquared();
      var distanceSqr2 = distanceSqr1;
      var i = 0;
      var p;
      var iter = 0;
      while (iter < k_maxIters) {
         saveCount = simplex.count;
         for (i = 0;
         i < saveCount; i++) {
            saveA[i] = vertices[i].indexA;
            saveB[i] = vertices[i].indexB;
         }
         switch (simplex.count) {
         case 1:
            break;
         case 2:
            simplex.Solve2();
            break;
         case 3:
            simplex.Solve3();
            break;
         default:
            Settings.Assert(false);
         }
         if (simplex.count == 3) {
            break;
         }
         p = simplex.GetClosestPoint();
         distanceSqr2 = p.LengthSquared();
         if (distanceSqr2 > distanceSqr1) {}
         distanceSqr1 = distanceSqr2;
         var d = simplex.GetSearchDirection();
         if (d.LengthSquared() < Number.MIN_VALUE * Number.MIN_VALUE) {
            break;
         }
         var vertex = vertices[simplex.count];
         vertex.indexA = proxyA.GetSupport(Math.MulTMV(transformA.R, d.GetNegative()));
         vertex.wA = Math.MulX(transformA, proxyA.GetVertex(vertex.indexA));
         vertex.indexB = proxyB.GetSupport(Math.MulTMV(transformB.R, d));
         vertex.wB = Math.MulX(transformB, proxyB.GetVertex(vertex.indexB));
         vertex.w = Math.SubtractVV(vertex.wB, vertex.wA);
         ++iter;
         ++Distance._gjkIters;
         var duplicate = false;
         for (i = 0;
         i < saveCount; i++) {
            if (vertex.indexA == saveA[i] && vertex.indexB == saveB[i]) {
               duplicate = true;
               break;
            }
         }
         if (duplicate) {
            break;
         }++simplex.count;
      }
      Distance._gjkMaxIters = Math.Max(Distance._gjkMaxIters, iter);
      simplex.GetWitnessPoints(output.pointA, output.pointB);
      output.distance = Math.SubtractVV(output.pointA, output.pointB).Length();
      output.iterations = iter;
      simplex.WriteCache(cache);
      if (input.useRadii) {
         var rA = proxyA.radius;
         var rB = proxyB.radius;
         if (output.distance > rA + rB && output.distance > Number.MIN_VALUE) {
            output.distance -= rA + rB;
            var normal = Math.SubtractVV(output.pointB, output.pointA);
            normal.Normalize();
            output.pointA.x += rA * normal.x;
            output.pointA.y += rA * normal.y;
            output.pointB.x -= rB * normal.x;
            output.pointB.y -= rB * normal.y;
         }
         else {
            p = new Vec2();
            p.x = .5 * (output.pointA.x + output.pointB.x);
            p.y = .5 * (output.pointA.y + output.pointB.y);
            output.pointA.x = output.pointB.x = p.x;
            output.pointA.y = output.pointB.y = p.y;
            output.distance = 0.0;
         }
      }
   }
   Box2D.postDefs.push(function () {
      Box2D.Collision.Distance.s_simplex = new Simplex();
      Box2D.Collision.Distance.s_saveA = new Vector_a2j_Number(3);
      Box2D.Collision.Distance.s_saveB = new Vector_a2j_Number(3);
   });
   DistanceInput.DistanceInput = function () {};
   DistanceOutput.DistanceOutput = function () {
      this.pointA = new Vec2();
      this.pointB = new Vec2();
   };
   DistanceProxy.DistanceProxy = function () {};
   DistanceProxy.prototype.Set = function (shape) {
      switch (shape.GetType()) {
      case Shape.e_circleShape:
         {
            var circle = (shape instanceof CircleShape ? shape : null);
            this.vertices = new Vector(1, true);
            this.vertices[0] = circle.p;
            this.count = 1;
            this.radius = circle.radius;
         }
         break;
      case Shape.e_polygonShape:
         {
            var polygon = (shape instanceof PolygonShape ? shape : null);
            this.vertices = polygon.vertices;
            this.count = polygon.vertexCount;
            this.radius = polygon.radius;
         }
         break;
      default:
         Settings.Assert(false);
      }
   }
   DistanceProxy.prototype.GetSupport = function (d) {
      var bestIndex = 0;
      var bestValue = this.vertices[0].x * d.x + this.vertices[0].y * d.y;
      for (var i = 1; i < this.count; ++i) {
         var value = this.vertices[i].x * d.x + this.vertices[i].y * d.y;
         if (value > bestValue) {
            bestIndex = i;
            bestValue = value;
         }
      }
      return bestIndex;
   }
   DistanceProxy.prototype.GetSupportVertex = function (d) {
      var bestIndex = 0;
      var bestValue = this.vertices[0].x * d.x + this.vertices[0].y * d.y;
      for (var i = 1; i < this.count; ++i) {
         var value = this.vertices[i].x * d.x + this.vertices[i].y * d.y;
         if (value > bestValue) {
            bestIndex = i;
            bestValue = value;
         }
      }
      return this.vertices[bestIndex];
   }
   DistanceProxy.prototype.GetVertexCount = function () {
      return this.count;
   }
   DistanceProxy.prototype.GetVertex = function (index) {
      if (index === undefined) index = 0;
      Settings.Assert(0 <= index && index < this.count);
      return this.vertices[index];
   }
   DynamicTree.DynamicTree = function () {};
   DynamicTree.prototype.DynamicTree = function () {
      this.root = null;
      this.freeList = null;
      this.path = 0;
      this.insertionCount = 0;
   }
   DynamicTree.prototype.CreateProxy = function (aabb, userData) {
      var node = this.AllocateNode();
      var extendX = Settings._aabbExtension;
      var extendY = Settings._aabbExtension;
      node.aabb.lowerBound.x = aabb.lowerBound.x - extendX;
      node.aabb.lowerBound.y = aabb.lowerBound.y - extendY;
      node.aabb.upperBound.x = aabb.upperBound.x + extendX;
      node.aabb.upperBound.y = aabb.upperBound.y + extendY;
      node.userData = userData;
      this.InsertLeaf(node);
      return node;
   }
   DynamicTree.prototype.DestroyProxy = function (proxy) {
      this.RemoveLeaf(proxy);
      this.FreeNode(proxy);
   }
   DynamicTree.prototype.MoveProxy = function (proxy, aabb, displacement) {
      Settings.Assert(proxy.IsLeaf());
      if (proxy.aabb.Contains(aabb)) {
         return false;
      }
      this.RemoveLeaf(proxy);
      var extendX = Settings._aabbExtension + Settings._aabbMultiplier * (displacement.x > 0 ? displacement.x : (-displacement.x));
      var extendY = Settings._aabbExtension + Settings._aabbMultiplier * (displacement.y > 0 ? displacement.y : (-displacement.y));
      proxy.aabb.lowerBound.x = aabb.lowerBound.x - extendX;
      proxy.aabb.lowerBound.y = aabb.lowerBound.y - extendY;
      proxy.aabb.upperBound.x = aabb.upperBound.x + extendX;
      proxy.aabb.upperBound.y = aabb.upperBound.y + extendY;
      this.InsertLeaf(proxy);
      return true;
   }
   DynamicTree.prototype.Rebalance = function (iterations) {
      if (iterations === undefined) iterations = 0;
      if (this.root == null) return;
      for (var i = 0; i < iterations; i++) {
         var node = this.root;
         var bit = 0;
         while (node.IsLeaf() == false) {
            node = (this.path >> bit) & 1 ? node.child2 : node.child1;
            bit = (bit + 1) & 31;
         }++this.path;
         this.RemoveLeaf(node);
         this.InsertLeaf(node);
      }
   }
   DynamicTree.prototype.GetFatAABB = function (proxy) {
      return proxy.aabb;
   }
   DynamicTree.prototype.GetUserData = function (proxy) {
      return proxy.userData;
   }
   DynamicTree.prototype.Query = function (callback, aabb) {
      if (this.root == null) return;
      var stack = new Vector();
      var count = 0;
      stack[count++] = this.root;
      while (count > 0) {
         var node = stack[--count];
         if (node.aabb.TestOverlap(aabb)) {
            if (node.IsLeaf()) {
               var proceed = callback(node);
               if (!proceed) return;
            }
            else {
               stack[count++] = node.child1;
               stack[count++] = node.child2;
            }
         }
      }
   }
   DynamicTree.prototype.RayCast = function (callback, input) {
      if (this.root == null) return;
      var p1 = input.p1;
      var p2 = input.p2;
      var r = Math.SubtractVV(p1, p2);
      r.Normalize();
      var v = Math.CrossFV(1.0, r);
      var abs_v = Math.AbsV(v);
      var maxFraction = input.maxFraction;
      var segmentAABB = new AABB();
      var tX = 0;
      var tY = 0; {
         tX = p1.x + maxFraction * (p2.x - p1.x);
         tY = p1.y + maxFraction * (p2.y - p1.y);
         segmentAABB.lowerBound.x = Math.min(p1.x, tX);
         segmentAABB.lowerBound.y = Math.min(p1.y, tY);
         segmentAABB.upperBound.x = Math.max(p1.x, tX);
         segmentAABB.upperBound.y = Math.max(p1.y, tY);
      }
      var stack = new Vector();
      var count = 0;
      stack[count++] = this.root;
      while (count > 0) {
         var node = stack[--count];
         if (node.aabb.TestOverlap(segmentAABB) == false) {
            continue;
         }
         var c = node.aabb.GetCenter();
         var h = node.aabb.GetExtents();
         var separation = Math.abs(v.x * (p1.x - c.x) + v.y * (p1.y - c.y)) - abs_v.x * h.x - abs_v.y * h.y;
         if (separation > 0.0) continue;
         if (node.IsLeaf()) {
            var subInput = new RayCastInput();
            subInput.p1 = input.p1;
            subInput.p2 = input.p2;
            subInput.maxFraction = input.maxFraction;
            maxFraction = callback(subInput, node);
            if (maxFraction == 0.0) return;
            if (maxFraction > 0.0) {
               tX = p1.x + maxFraction * (p2.x - p1.x);
               tY = p1.y + maxFraction * (p2.y - p1.y);
               segmentAABB.lowerBound.x = Math.min(p1.x, tX);
               segmentAABB.lowerBound.y = Math.min(p1.y, tY);
               segmentAABB.upperBound.x = Math.max(p1.x, tX);
               segmentAABB.upperBound.y = Math.max(p1.y, tY);
            }
         }
         else {
            stack[count++] = node.child1;
            stack[count++] = node.child2;
         }
      }
   }
   DynamicTree.prototype.AllocateNode = function () {
      if (this.freeList) {
         var node = this.freeList;
         this.freeList = node.parent;
         node.parent = null;
         node.child1 = null;
         node.child2 = null;
         return node;
      }
      return new DynamicTreeNode();
   }
   DynamicTree.prototype.FreeNode = function (node) {
      node.parent = this.freeList;
      this.freeList = node;
   }
   DynamicTree.prototype.InsertLeaf = function (leaf) {
      ++this.insertionCount;
      if (this.root == null) {
         this.root = leaf;
         this.root.parent = null;
         return;
      }
      var center = leaf.aabb.GetCenter();
      var sibling = this.root;
      if (sibling.IsLeaf() == false) {
         do {
            var child1 = sibling.child1;
            var child2 = sibling.child2;
            var norm1 = Math.abs((child1.aabb.lowerBound.x + child1.aabb.upperBound.x) / 2 - center.x) + Math.abs((child1.aabb.lowerBound.y + child1.aabb.upperBound.y) / 2 - center.y);
            var norm2 = Math.abs((child2.aabb.lowerBound.x + child2.aabb.upperBound.x) / 2 - center.x) + Math.abs((child2.aabb.lowerBound.y + child2.aabb.upperBound.y) / 2 - center.y);
            if (norm1 < norm2) {
               sibling = child1;
            }
            else {
               sibling = child2;
            }
         }
         while (sibling.IsLeaf() == false)
      }
      var node1 = sibling.parent;
      var node2 = this.AllocateNode();
      node2.parent = node1;
      node2.userData = null;
      node2.aabb.Combine(leaf.aabb, sibling.aabb);
      if (node1) {
         if (sibling.parent.child1 == sibling) {
            node1.child1 = node2;
         }
         else {
            node1.child2 = node2;
         }
         node2.child1 = sibling;
         node2.child2 = leaf;
         sibling.parent = node2;
         leaf.parent = node2;
         do {
            if (node1.aabb.Contains(node2.aabb)) break;
            node1.aabb.Combine(node1.child1.aabb, node1.child2.aabb);
            node2 = node1;
            node1 = node1.parent;
         }
         while (node1)
      }
      else {
         node2.child1 = sibling;
         node2.child2 = leaf;
         sibling.parent = node2;
         leaf.parent = node2;
         this.root = node2;
      }
   }
   DynamicTree.prototype.RemoveLeaf = function (leaf) {
      if (leaf == this.root) {
         this.root = null;
         return;
      }
      var node2 = leaf.parent;
      var node1 = node2.parent;
      var sibling;
      if (node2.child1 == leaf) {
         sibling = node2.child2;
      }
      else {
         sibling = node2.child1;
      }
      if (node1) {
         if (node1.child1 == node2) {
            node1.child1 = sibling;
         }
         else {
            node1.child2 = sibling;
         }
         sibling.parent = node1;
         this.FreeNode(node2);
         while (node1) {
            var oldAABB = node1.aabb;
            node1.aabb = AABB.Combine(node1.child1.aabb, node1.child2.aabb);
            if (oldAABB.Contains(node1.aabb)) break;
            node1 = node1.parent;
         }
      }
      else {
         this.root = sibling;
         sibling.parent = null;
         this.FreeNode(node2);
      }
   }
   DynamicTreeBroadPhase.DynamicTreeBroadPhase = function () {
      this.tree = new DynamicTree();
      this.moveBuffer = new Vector();
      this.pairBuffer = new Vector();
      this.pairCount = 0;
   };
   DynamicTreeBroadPhase.prototype.CreateProxy = function (aabb, userData) {
      var proxy = this.tree.CreateProxy(aabb, userData);
      ++this.proxyCount;
      this.BufferMove(proxy);
      return proxy;
   }
   DynamicTreeBroadPhase.prototype.DestroyProxy = function (proxy) {
      this.UnBufferMove(proxy);
      --this.proxyCount;
      this.tree.DestroyProxy(proxy);
   }
   DynamicTreeBroadPhase.prototype.MoveProxy = function (proxy, aabb, displacement) {
      var buffer = this.tree.MoveProxy(proxy, aabb, displacement);
      if (buffer) {
         this.BufferMove(proxy);
      }
   }
   DynamicTreeBroadPhase.prototype.TestOverlap = function (proxyA, proxyB) {
      var aabbA = this.tree.GetFatAABB(proxyA);
      var aabbB = this.tree.GetFatAABB(proxyB);
      return aabbA.TestOverlap(aabbB);
   }
   DynamicTreeBroadPhase.prototype.GetUserData = function (proxy) {
      return this.tree.GetUserData(proxy);
   }
   DynamicTreeBroadPhase.prototype.GetFatAABB = function (proxy) {
      return this.tree.GetFatAABB(proxy);
   }
   DynamicTreeBroadPhase.prototype.GetProxyCount = function () {
      return this.proxyCount;
   }
   DynamicTreeBroadPhase.prototype.UpdatePairs = function (callback) {
      var __this = this;
      __this.pairCount = 0;
      var i = 0,
         queryProxy;
      for (i = 0;
      i < __this.moveBuffer.length; ++i) {
         queryProxy = __this.moveBuffer[i];

         function QueryCallback(proxy) {
            if (proxy == queryProxy) return true;
            if (__this.pairCount == __this.pairBuffer.length) {
               __this.pairBuffer[__this.pairCount] = new DynamicTreePair();
            }
            var pair = __this.pairBuffer[__this.pairCount];
            pair.proxyA = proxy < queryProxy ? proxy : queryProxy;
            pair.proxyB = proxy >= queryProxy ? proxy : queryProxy;++__this.pairCount;
            return true;
         };
         var fatAABB = __this.tree.GetFatAABB(queryProxy);
         __this.tree.Query(QueryCallback, fatAABB);
      }
      __this.moveBuffer.length = 0;
      for (var i = 0; i < __this.pairCount;) {
         var primaryPair = __this.pairBuffer[i];
         var userDataA = __this.tree.GetUserData(primaryPair.proxyA);
         var userDataB = __this.tree.GetUserData(primaryPair.proxyB);
         callback(userDataA, userDataB);
         ++i;
         while (i < __this.pairCount) {
            var pair = __this.pairBuffer[i];
            if (pair.proxyA != primaryPair.proxyA || pair.proxyB != primaryPair.proxyB) {
               break;
            }++i;
         }
      }
   }
   DynamicTreeBroadPhase.prototype.Query = function (callback, aabb) {
      this.tree.Query(callback, aabb);
   }
   DynamicTreeBroadPhase.prototype.RayCast = function (callback, input) {
      this.tree.RayCast(callback, input);
   }
   DynamicTreeBroadPhase.prototype.Validate = function () {}
   DynamicTreeBroadPhase.prototype.Rebalance = function (iterations) {
      if (iterations === undefined) iterations = 0;
      this.tree.Rebalance(iterations);
   }
   DynamicTreeBroadPhase.prototype.BufferMove = function (proxy) {
      this.moveBuffer[this.moveBuffer.length] = proxy;
   }
   DynamicTreeBroadPhase.prototype.UnBufferMove = function (proxy) {
      var i = parseInt(this.moveBuffer.indexOf(proxy));
      this.moveBuffer.splice(i, 1);
   }
   DynamicTreeBroadPhase.prototype.ComparePairs = function (pair1, pair2) {
      return 0;
   }
   DynamicTreeBroadPhase.__implements = {};
   DynamicTreeBroadPhase.__implements[IBroadPhase] = true;
   DynamicTreeNode.DynamicTreeNode = function () {
      this.aabb = new AABB();
   };
   DynamicTreeNode.prototype.IsLeaf = function () {
      return this.child1 == null;
   }
   DynamicTreePair.DynamicTreePair = function () {};
   Manifold.Manifold = function () {
      this.pointCount = 0;
   };
   Manifold.prototype.Manifold = function () {
      this.points = new Vector(Settings._maxManifoldPoints);
      for (var i = 0; i < Settings._maxManifoldPoints; i++) {
         this.points[i] = new ManifoldPoint();
      }
      this.localPlaneNormal = new Vec2();
      this.localPoint = new Vec2();
   }
   Manifold.prototype.Reset = function () {
      for (var i = 0; i < Settings._maxManifoldPoints; i++) {
         ((this.points[i] instanceof ManifoldPoint ? this.points[i] : null)).Reset();
      }
      this.localPlaneNormal.SetZero();
      this.localPoint.SetZero();
      this.type = 0;
      this.pointCount = 0;
   }
   Manifold.prototype.Set = function (m) {
      this.pointCount = m.pointCount;
      for (var i = 0; i < Settings._maxManifoldPoints; i++) {
         ((this.points[i] instanceof ManifoldPoint ? this.points[i] : null)).Set(m.points[i]);
      }
      this.localPlaneNormal.SetV(m.localPlaneNormal);
      this.localPoint.SetV(m.localPoint);
      this.type = m.type;
   }
   Manifold.prototype.Copy = function () {
      var copy = new Manifold();
      copy.Set(this);
      return copy;
   }
   Box2D.postDefs.push(function () {
      Box2D.Collision.Manifold.e_circles = 0x0001;
      Box2D.Collision.Manifold.e_faceA = 0x0002;
      Box2D.Collision.Manifold.e_faceB = 0x0004;
   });
   ManifoldPoint.ManifoldPoint = function () {
      this.localPoint = new Vec2();
      this.id = new ContactID();
   };
   ManifoldPoint.prototype.ManifoldPoint = function () {
      this.Reset();
   }
   ManifoldPoint.prototype.Reset = function () {
      this.localPoint.SetZero();
      this.normalImpulse = 0.0;
      this.tangentImpulse = 0.0;
      this.id.key = 0;
   }
   ManifoldPoint.prototype.Set = function (m) {
      this.localPoint.SetV(m.localPoint);
      this.normalImpulse = m.normalImpulse;
      this.tangentImpulse = m.tangentImpulse;
      this.id.Set(m.id);
   }
   Point.Point = function () {
      this.p = new Vec2();
   };
   Point.prototype.Support = function (xf, vX, vY) {
      if (vX === undefined) vX = 0;
      if (vY === undefined) vY = 0;
      return this.p;
   }
   Point.prototype.GetFirstVertex = function (xf) {
      return this.p;
   }
   RayCastInput.RayCastInput = function () {
      this.p1 = new Vec2();
      this.p2 = new Vec2();
   };
   RayCastInput.prototype.RayCastInput = function (p1, p2, maxFraction) {
      if (p1 === undefined) p1 = null;
      if (p2 === undefined) p2 = null;
      if (maxFraction === undefined) maxFraction = 1;
      if (p1) this.p1.SetV(p1);
      if (p2) this.p2.SetV(p2);
      this.maxFraction = maxFraction;
   }
   RayCastOutput.RayCastOutput = function () {
      this.normal = new Vec2();
   };
   Segment.Segment = function () {
      this.p1 = new Vec2();
      this.p2 = new Vec2();
   };
   Segment.prototype.TestSegment = function (lambda, normal, segment, maxLambda) {
      if (maxLambda === undefined) maxLambda = 0;
      var s = segment.p1;
      var rX = segment.p2.x - s.x;
      var rY = segment.p2.y - s.y;
      var dX = this.p2.x - this.p1.x;
      var dY = this.p2.y - this.p1.y;
      var nX = dY;
      var nY = (-dX);
      var k_slop = 100.0 * Number.MIN_VALUE;
      var denom = (-(rX * nX + rY * nY));
      if (denom > k_slop) {
         var bX = s.x - this.p1.x;
         var bY = s.y - this.p1.y;
         var a = (bX * nX + bY * nY);
         if (0.0 <= a && a <= maxLambda * denom) {
            var mu2 = (-rX * bY) + rY * bX;
            if ((-k_slop * denom) <= mu2 && mu2 <= denom * (1.0 + k_slop)) {
               a /= denom;
               var nLen = Math.sqrt(nX * nX + nY * nY);
               nX /= nLen;
               nY /= nLen;
               lambda[0] = a;
               normal.Set(nX, nY);
               return true;
            }
         }
      }
      return false;
   }
   Segment.prototype.Extend = function (aabb) {
      this.ExtendForward(aabb);
      this.ExtendBackward(aabb);
   }
   Segment.prototype.ExtendForward = function (aabb) {
      var dX = this.p2.x - this.p1.x;
      var dY = this.p2.y - this.p1.y;
      var lambda = Math.min(dX > 0 ? (aabb.upperBound.x - this.p1.x) / dX : dX < 0 ? (aabb.lowerBound.x - this.p1.x) / dX : Number.POSITIVE_INFINITY,
      dY > 0 ? (aabb.upperBound.y - this.p1.y) / dY : dY < 0 ? (aabb.lowerBound.y - this.p1.y) / dY : Number.POSITIVE_INFINITY);
      this.p2.x = this.p1.x + dX * lambda;
      this.p2.y = this.p1.y + dY * lambda;
   }
   Segment.prototype.ExtendBackward = function (aabb) {
      var dX = (-this.p2.x) + this.p1.x;
      var dY = (-this.p2.y) + this.p1.y;
      var lambda = Math.min(dX > 0 ? (aabb.upperBound.x - this.p2.x) / dX : dX < 0 ? (aabb.lowerBound.x - this.p2.x) / dX : Number.POSITIVE_INFINITY,
      dY > 0 ? (aabb.upperBound.y - this.p2.y) / dY : dY < 0 ? (aabb.lowerBound.y - this.p2.y) / dY : Number.POSITIVE_INFINITY);
      this.p1.x = this.p2.x + dX * lambda;
      this.p1.y = this.p2.y + dY * lambda;
   }
   SeparationFunction.SeparationFunction = function () {
      this.localPoint = new Vec2();
      this.axis = new Vec2();
   };
   SeparationFunction.prototype.Initialize = function (cache, proxyA, transformA, proxyB, transformB) {
      this.proxyA = proxyA;
      this.proxyB = proxyB;
      var count = parseInt(cache.count);
      Settings.Assert(0 < count && count < 3);
      var localPointA;
      var localPointA1;
      var localPointA2;
      var localPointB;
      var localPointB1;
      var localPoint;
      var pointAX = 0;
      var pointAY = 0;
      var pointBX = 0;
      var pointBY = 0;
      var normalX = 0;
      var normalY = 0;
      var tMat;
      var tVec;
      var s = 0;
      var sgn = 0;
      if (count == 1) {
         this.type = SeparationFunction.e_points;
         localPointA = this.proxyA.GetVertex(cache.indexA[0]);
         localPointB = this.proxyB.GetVertex(cache.indexB[0]);
         tVec = localPointA;
         tMat = transformA.R;
         pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
         pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
         tVec = localPointB;
         tMat = transformB.R;
         pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
         pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
         this.axis.x = pointBX - pointAX;
         this.axis.y = pointBY - pointAY;
         this.axis.Normalize();
      }
      else if (cache.indexB[0] == cache.indexB[1]) {
         this.type = SeparationFunction.e_faceA;
         localPointA1 = this.proxyA.GetVertex(cache.indexA[0]);
         localPointA2 = this.proxyA.GetVertex(cache.indexA[1]);
         localPointB = this.proxyB.GetVertex(cache.indexB[0]);
         this.localPoint.x = 0.5 * (localPointA1.x + localPointA2.x);
         this.localPoint.y = 0.5 * (localPointA1.y + localPointA2.y);
         this.axis = Math.CrossVF(Math.SubtractVV(localPointA2, localPointA1), 1.0);
         this.axis.Normalize();
         tVec = this.axis;
         tMat = transformA.R;
         normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
         normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
         tVec = this.localPoint;
         tMat = transformA.R;
         pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
         pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
         tVec = localPointB;
         tMat = transformB.R;
         pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
         pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
         s = (pointBX - pointAX) * normalX + (pointBY - pointAY) * normalY;
         if (s < 0.0) {
            this.axis.NegativeSelf();
         }
      }
      else if (cache.indexA[0] == cache.indexA[0]) {
         this.type = SeparationFunction.e_faceB;
         localPointB1 = this.proxyB.GetVertex(cache.indexB[0]);
         localPoint = this.proxyB.GetVertex(cache.indexB[1]);
         localPointA = this.proxyA.GetVertex(cache.indexA[0]);
         this.localPoint.x = 0.5 * (localPointB1.x + localPoint.x);
         this.localPoint.y = 0.5 * (localPointB1.y + localPoint.y);
         this.axis = Math.CrossVF(Math.SubtractVV(localPoint, localPointB1), 1.0);
         this.axis.Normalize();
         tVec = this.axis;
         tMat = transformB.R;
         normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
         normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
         tVec = this.localPoint;
         tMat = transformB.R;
         pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
         pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
         tVec = localPointA;
         tMat = transformA.R;
         pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
         pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
         s = (pointAX - pointBX) * normalX + (pointAY - pointBY) * normalY;
         if (s < 0.0) {
            this.axis.NegativeSelf();
         }
      }
      else {
         localPointA1 = this.proxyA.GetVertex(cache.indexA[0]);
         localPointA2 = this.proxyA.GetVertex(cache.indexA[1]);
         localPointB1 = this.proxyB.GetVertex(cache.indexB[0]);
         localPoint = this.proxyB.GetVertex(cache.indexB[1]);
         var pA = Math.MulX(transformA, localPointA);
         var dA = Math.MulMV(transformA.R, Math.SubtractVV(localPointA2, localPointA1));
         var pB = Math.MulX(transformB, localPointB);
         var dB = Math.MulMV(transformB.R, Math.SubtractVV(localPoint, localPointB1));
         var a = dA.x * dA.x + dA.y * dA.y;
         var e = dB.x * dB.x + dB.y * dB.y;
         var r = Math.SubtractVV(dB, dA);
         var c = dA.x * r.x + dA.y * r.y;
         var f = dB.x * r.x + dB.y * r.y;
         var b = dA.x * dB.x + dA.y * dB.y;
         var denom = a * e - b * b;
         s = 0.0;
         if (denom != 0.0) {
            s = Math.Clamp((b * f - c * e) / denom, 0.0, 1.0);
         }
         var t = (b * s + f) / e;
         if (t < 0.0) {
            t = 0.0;
            s = Math.Clamp((b - c) / a, 0.0, 1.0);
         }
         localPointA = new Vec2();
         localPointA.x = localPointA1.x + s * (localPointA2.x - localPointA1.x);
         localPointA.y = localPointA1.y + s * (localPointA2.y - localPointA1.y);
         localPointB = new Vec2();
         localPointB.x = localPointB1.x + s * (localPoint.x - localPointB1.x);
         localPointB.y = localPointB1.y + s * (localPoint.y - localPointB1.y);
         if (s == 0.0 || s == 1.0) {
            this.type = SeparationFunction.e_faceB;
            this.axis = Math.CrossVF(Math.SubtractVV(localPoint, localPointB1), 1.0);
            this.axis.Normalize();
            this.localPoint = localPointB;
            tVec = this.axis;
            tMat = transformB.R;
            normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            tVec = this.localPoint;
            tMat = transformB.R;
            pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
            pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
            tVec = localPointA;
            tMat = transformA.R;
            pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
            pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
            sgn = (pointAX - pointBX) * normalX + (pointAY - pointBY) * normalY;
            if (s < 0.0) {
               this.axis.NegativeSelf();
            }
         }
         else {
            this.type = SeparationFunction.e_faceA;
            this.axis = Math.CrossVF(Math.SubtractVV(localPointA2, localPointA1), 1.0);
            this.localPoint = localPointA;
            tVec = this.axis;
            tMat = transformA.R;
            normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            tVec = this.localPoint;
            tMat = transformA.R;
            pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
            pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
            tVec = localPointB;
            tMat = transformB.R;
            pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
            pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
            sgn = (pointBX - pointAX) * normalX + (pointBY - pointAY) * normalY;
            if (s < 0.0) {
               this.axis.NegativeSelf();
            }
         }
      }
   }
   SeparationFunction.prototype.Evaluate = function (transformA, transformB) {
      var axisA;
      var axisB;
      var localPointA;
      var localPointB;
      var pointA;
      var pointB;
      var seperation = 0;
      var normal;
      switch (this.type) {
      case SeparationFunction.e_points:
         {
            axisA = Math.MulTMV(transformA.R, this.axis);
            axisB = Math.MulTMV(transformB.R, this.axis.GetNegative());
            localPointA = this.proxyA.GetSupportVertex(axisA);
            localPointB = this.proxyB.GetSupportVertex(axisB);
            pointA = Math.MulX(transformA, localPointA);
            pointB = Math.MulX(transformB, localPointB);
            seperation = (pointB.x - pointA.x) * this.axis.x + (pointB.y - pointA.y) * this.axis.y;
            return seperation;
         }
      case SeparationFunction.e_faceA:
         {
            normal = Math.MulMV(transformA.R, this.axis);
            pointA = Math.MulX(transformA, this.localPoint);
            axisB = Math.MulTMV(transformB.R, normal.GetNegative());
            localPointB = this.proxyB.GetSupportVertex(axisB);
            pointB = Math.MulX(transformB, localPointB);
            seperation = (pointB.x - pointA.x) * normal.x + (pointB.y - pointA.y) * normal.y;
            return seperation;
         }
      case SeparationFunction.e_faceB:
         {
            normal = Math.MulMV(transformB.R, this.axis);
            pointB = Math.MulX(transformB, this.localPoint);
            axisA = Math.MulTMV(transformA.R, normal.GetNegative());
            localPointA = this.proxyA.GetSupportVertex(axisA);
            pointA = Math.MulX(transformA, localPointA);
            seperation = (pointA.x - pointB.x) * normal.x + (pointA.y - pointB.y) * normal.y;
            return seperation;
         }
      default:
         Settings.Assert(false);
         return 0.0;
      }
   }
   Box2D.postDefs.push(function () {
      Box2D.Collision.SeparationFunction.e_points = 0x01;
      Box2D.Collision.SeparationFunction.e_faceA = 0x02;
      Box2D.Collision.SeparationFunction.e_faceB = 0x04;
   });
   Simplex.Simplex = function () {
      this.v1 = new SimplexVertex();
      this.v2 = new SimplexVertex();
      this.v3 = new SimplexVertex();
      this.vertices = new Vector(3);
   };
   Simplex.prototype.Simplex = function () {
      this.vertices[0] = this.v1;
      this.vertices[1] = this.v2;
      this.vertices[2] = this.v3;
   }
   Simplex.prototype.ReadCache = function (cache, proxyA, transformA, proxyB, transformB) {
      Settings.Assert(0 <= cache.count && cache.count <= 3);
      var wALocal;
      var wBLocal;
      this.count = cache.count;
      var vertices = this.vertices;
      for (var i = 0; i < this.count; i++) {
         var v = vertices[i];
         v.indexA = cache.indexA[i];
         v.indexB = cache.indexB[i];
         wALocal = proxyA.GetVertex(v.indexA);
         wBLocal = proxyB.GetVertex(v.indexB);
         v.wA = Math.MulX(transformA, wALocal);
         v.wB = Math.MulX(transformB, wBLocal);
         v.w = Math.SubtractVV(v.wB, v.wA);
         v.a = 0;
      }
      if (this.count > 1) {
         var metric1 = cache.metric;
         var metric2 = this.GetMetric();
         if (metric2 < .5 * metric1 || 2.0 * metric1 < metric2 || metric2 < Number.MIN_VALUE) {
            this.count = 0;
         }
      }
      if (this.count == 0) {
         v = vertices[0];
         v.indexA = 0;
         v.indexB = 0;
         wALocal = proxyA.GetVertex(0);
         wBLocal = proxyB.GetVertex(0);
         v.wA = Math.MulX(transformA, wALocal);
         v.wB = Math.MulX(transformB, wBLocal);
         v.w = Math.SubtractVV(v.wB, v.wA);
         this.count = 1;
      }
   }
   Simplex.prototype.WriteCache = function (cache) {
      cache.metric = this.GetMetric();
      cache.count = Box2D.parseUInt(this.count);
      var vertices = this.vertices;
      for (var i = 0; i < this.count; i++) {
         cache.indexA[i] = Box2D.parseUInt(vertices[i].indexA);
         cache.indexB[i] = Box2D.parseUInt(vertices[i].indexB);
      }
   }
   Simplex.prototype.GetSearchDirection = function () {
      switch (this.count) {
      case 1:
         return this.v1.w.GetNegative();
      case 2:
         {
            var e12 = Math.SubtractVV(this.v2.w, this.v1.w);
            var sgn = Math.CrossVV(e12, this.v1.w.GetNegative());
            if (sgn > 0.0) {
               return Math.CrossFV(1.0, e12);
            }
            else {
               return Math.CrossVF(e12, 1.0);
            }
         }
      default:
         Settings.Assert(false);
         return new Vec2();
      }
   }
   Simplex.prototype.GetClosestPoint = function () {
      switch (this.count) {
      case 0:
         Settings.Assert(false);
         return new Vec2();
      case 1:
         return this.v1.w;
      case 2:
         return new Vec2(this.v1.a * this.v1.w.x + this.v2.a * this.v2.w.x, this.v1.a * this.v1.w.y + this.v2.a * this.v2.w.y);
      default:
         Settings.Assert(false);
         return new Vec2();
      }
   }
   Simplex.prototype.GetWitnessPoints = function (pA, pB) {
      switch (this.count) {
      case 0:
         Settings.Assert(false);
         break;
      case 1:
         pA.SetV(this.v1.wA);
         pB.SetV(this.v1.wB);
         break;
      case 2:
         pA.x = this.v1.a * this.v1.wA.x + this.v2.a * this.v2.wA.x;
         pA.y = this.v1.a * this.v1.wA.y + this.v2.a * this.v2.wA.y;
         pB.x = this.v1.a * this.v1.wB.x + this.v2.a * this.v2.wB.x;
         pB.y = this.v1.a * this.v1.wB.y + this.v2.a * this.v2.wB.y;
         break;
      case 3:
         pB.x = pA.x = this.v1.a * this.v1.wA.x + this.v2.a * this.v2.wA.x + this.v3.a * this.v3.wA.x;
         pB.y = pA.y = this.v1.a * this.v1.wA.y + this.v2.a * this.v2.wA.y + this.v3.a * this.v3.wA.y;
         break;
      default:
         Settings.Assert(false);
         break;
      }
   }
   Simplex.prototype.GetMetric = function () {
      switch (this.count) {
      case 0:
         Settings.Assert(false);
         return 0.0;
      case 1:
         return 0.0;
      case 2:
         return Math.SubtractVV(this.v1.w, this.v2.w).Length();
      case 3:
         return Math.CrossVV(Math.SubtractVV(this.v2.w, this.v1.w), Math.SubtractVV(this.v3.w, this.v1.w));
      default:
         Settings.Assert(false);
         return 0.0;
      }
   }
   Simplex.prototype.Solve2 = function () {
      var w1 = this.v1.w;
      var w2 = this.v2.w;
      var e12 = Math.SubtractVV(w2, w1);
      var d12_2 = (-(w1.x * e12.x + w1.y * e12.y));
      if (d12_2 <= 0.0) {
         this.v1.a = 1.0;
         this.count = 1;
         return;
      }
      var d12_1 = (w2.x * e12.x + w2.y * e12.y);
      if (d12_1 <= 0.0) {
         this.v2.a = 1.0;
         this.count = 1;
         this.v1.Set(this.v2);
         return;
      }
      var inv_d12 = 1.0 / (d12_1 + d12_2);
      this.v1.a = d12_1 * inv_d12;
      this.v2.a = d12_2 * inv_d12;
      this.count = 2;
   }
   Simplex.prototype.Solve3 = function () {
      var w1 = this.v1.w;
      var w2 = this.v2.w;
      var w3 = this.v3.w;
      var e12 = Math.SubtractVV(w2, w1);
      var w1e12 = Math.Dot(w1, e12);
      var w2e12 = Math.Dot(w2, e12);
      var d12_1 = w2e12;
      var d12_2 = (-w1e12);
      var e13 = Math.SubtractVV(w3, w1);
      var w1e13 = Math.Dot(w1, e13);
      var w3e13 = Math.Dot(w3, e13);
      var d13_1 = w3e13;
      var d13_2 = (-w1e13);
      var e23 = Math.SubtractVV(w3, w2);
      var w2e23 = Math.Dot(w2, e23);
      var w3e23 = Math.Dot(w3, e23);
      var d23_1 = w3e23;
      var d23_2 = (-w2e23);
      var n123 = Math.CrossVV(e12, e13);
      var d123_1 = n123 * Math.CrossVV(w2, w3);
      var d123_2 = n123 * Math.CrossVV(w3, w1);
      var d123_3 = n123 * Math.CrossVV(w1, w2);
      if (d12_2 <= 0.0 && d13_2 <= 0.0) {
         this.v1.a = 1.0;
         this.count = 1;
         return;
      }
      if (d12_1 > 0.0 && d12_2 > 0.0 && d123_3 <= 0.0) {
         var inv_d12 = 1.0 / (d12_1 + d12_2);
         this.v1.a = d12_1 * inv_d12;
         this.v2.a = d12_2 * inv_d12;
         this.count = 2;
         return;
      }
      if (d13_1 > 0.0 && d13_2 > 0.0 && d123_2 <= 0.0) {
         var inv_d13 = 1.0 / (d13_1 + d13_2);
         this.v1.a = d13_1 * inv_d13;
         this.v3.a = d13_2 * inv_d13;
         this.count = 2;
         this.v2.Set(this.v3);
         return;
      }
      if (d12_1 <= 0.0 && d23_2 <= 0.0) {
         this.v2.a = 1.0;
         this.count = 1;
         this.v1.Set(this.v2);
         return;
      }
      if (d13_1 <= 0.0 && d23_1 <= 0.0) {
         this.v3.a = 1.0;
         this.count = 1;
         this.v1.Set(this.v3);
         return;
      }
      if (d23_1 > 0.0 && d23_2 > 0.0 && d123_1 <= 0.0) {
         var inv_d23 = 1.0 / (d23_1 + d23_2);
         this.v2.a = d23_1 * inv_d23;
         this.v3.a = d23_2 * inv_d23;
         this.count = 2;
         this.v1.Set(this.v3);
         return;
      }
      var inv_d123 = 1.0 / (d123_1 + d123_2 + d123_3);
      this.v1.a = d123_1 * inv_d123;
      this.v2.a = d123_2 * inv_d123;
      this.v3.a = d123_3 * inv_d123;
      this.count = 3;
   }
   SimplexCache.SimplexCache = function () {
      this.indexA = new Vector_a2j_Number(3);
      this.indexB = new Vector_a2j_Number(3);
   };
   SimplexVertex.SimplexVertex = function () {};
   SimplexVertex.prototype.Set = function (other) {
      this.wA.SetV(other.wA);
      this.wB.SetV(other.wB);
      this.w.SetV(other.w);
      this.a = other.a;
      this.indexA = other.indexA;
      this.indexB = other.indexB;
   }
   TimeOfImpact.TimeOfImpact = function () {};
   TimeOfImpact.TimeOfImpact = function (input) {
      ++TimeOfImpact._toiCalls;
      var proxyA = input.proxyA;
      var proxyB = input.proxyB;
      var sweepA = input.sweepA;
      var sweepB = input.sweepB;
      Settings.Assert(sweepA.t0 == sweepB.t0);
      Settings.Assert(1.0 - sweepA.t0 > Number.MIN_VALUE);
      var radius = proxyA.radius + proxyB.radius;
      var tolerance = input.tolerance;
      var alpha = 0.0;
      var k_maxIterations = 1000;
      var iter = 0;
      var target = 0.0;
      TimeOfImpact.s_cache.count = 0;
      TimeOfImpact.s_distanceInput.useRadii = false;
      for (;;) {
         sweepA.GetTransform(TimeOfImpact.s_xfA, alpha);
         sweepB.GetTransform(TimeOfImpact.s_xfB, alpha);
         TimeOfImpact.s_distanceInput.proxyA = proxyA;
         TimeOfImpact.s_distanceInput.proxyB = proxyB;
         TimeOfImpact.s_distanceInput.transformA = TimeOfImpact.s_xfA;
         TimeOfImpact.s_distanceInput.transformB = TimeOfImpact.s_xfB;
         Distance.Distance(TimeOfImpact.s_distanceOutput, TimeOfImpact.s_cache, TimeOfImpact.s_distanceInput);
         if (TimeOfImpact.s_distanceOutput.distance <= 0.0) {
            alpha = 1.0;
            break;
         }
         TimeOfImpact.s_fcn.Initialize(TimeOfImpact.s_cache, proxyA, TimeOfImpact.s_xfA, proxyB, TimeOfImpact.s_xfB);
         var separation = TimeOfImpact.s_fcn.Evaluate(TimeOfImpact.s_xfA, TimeOfImpact.s_xfB);
         if (separation <= 0.0) {
            alpha = 1.0;
            break;
         }
         if (iter == 0) {
            if (separation > radius) {
               target = Math.Max(radius - tolerance, 0.75 * radius);
            }
            else {
               target = Math.Max(separation - tolerance, 0.02 * radius);
            }
         }
         if (separation - target < 0.5 * tolerance) {
            if (iter == 0) {
               alpha = 1.0;
               break;
            }
            break;
         }
         var newAlpha = alpha; {
            var x1 = alpha;
            var x2 = 1.0;
            var f1 = separation;
            sweepA.GetTransform(TimeOfImpact.s_xfA, x2);
            sweepB.GetTransform(TimeOfImpact.s_xfB, x2);
            var f2 = TimeOfImpact.s_fcn.Evaluate(TimeOfImpact.s_xfA, TimeOfImpact.s_xfB);
            if (f2 >= target) {
               alpha = 1.0;
               break;
            }
            var rootIterCount = 0;
            for (;;) {
               var x = 0;
               if (rootIterCount & 1) {
                  x = x1 + (target - f1) * (x2 - x1) / (f2 - f1);
               }
               else {
                  x = 0.5 * (x1 + x2);
               }
               sweepA.GetTransform(TimeOfImpact.s_xfA, x);
               sweepB.GetTransform(TimeOfImpact.s_xfB, x);
               var f = TimeOfImpact.s_fcn.Evaluate(TimeOfImpact.s_xfA, TimeOfImpact.s_xfB);
               if (Math.Abs(f - target) < 0.025 * tolerance) {
                  newAlpha = x;
                  break;
               }
               if (f > target) {
                  x1 = x;
                  f1 = f;
               }
               else {
                  x2 = x;
                  f2 = f;
               }++rootIterCount;
               ++TimeOfImpact._toiRootIters;
               if (rootIterCount == 50) {
                  break;
               }
            }
            TimeOfImpact._toiMaxRootIters = Math.Max(TimeOfImpact._toiMaxRootIters, rootIterCount);
         }
         if (newAlpha < (1.0 + 100.0 * Number.MIN_VALUE) * alpha) {
            break;
         }
         alpha = newAlpha;
         iter++;
         ++TimeOfImpact._toiIters;
         if (iter == k_maxIterations) {
            break;
         }
      }
      TimeOfImpact._toiMaxIters = Math.Max(TimeOfImpact._toiMaxIters, iter);
      return alpha;
   }
   Box2D.postDefs.push(function () {
      Box2D.Collision.TimeOfImpact._toiCalls = 0;
      Box2D.Collision.TimeOfImpact._toiIters = 0;
      Box2D.Collision.TimeOfImpact._toiMaxIters = 0;
      Box2D.Collision.TimeOfImpact._toiRootIters = 0;
      Box2D.Collision.TimeOfImpact._toiMaxRootIters = 0;
      Box2D.Collision.TimeOfImpact.s_cache = new SimplexCache();
      Box2D.Collision.TimeOfImpact.s_distanceInput = new DistanceInput();
      Box2D.Collision.TimeOfImpact.s_xfA = new Transform();
      Box2D.Collision.TimeOfImpact.s_xfB = new Transform();
      Box2D.Collision.TimeOfImpact.s_fcn = new SeparationFunction();
      Box2D.Collision.TimeOfImpact.s_distanceOutput = new DistanceOutput();
   });
   TOIInput.TOIInput = function () {
      this.proxyA = new DistanceProxy();
      this.proxyB = new DistanceProxy();
      this.sweepA = new Sweep();
      this.sweepB = new Sweep();
   };
   WorldManifold.WorldManifold = function () {
      this.normal = new Vec2();
   };
   WorldManifold.prototype.WorldManifold = function () {
      this.points = new Vector(Settings._maxManifoldPoints);
      for (var i = 0; i < Settings._maxManifoldPoints; i++) {
         this.points[i] = new Vec2();
      }
   }
   WorldManifold.prototype.Initialize = function (manifold, xfA, radiusA, xfB, radiusB) {
      if (radiusA === undefined) radiusA = 0;
      if (radiusB === undefined) radiusB = 0;
      if (manifold.pointCount == 0) {
         return;
      }
      var i = 0;
      var tVec;
      var tMat;
      var normalX = 0;
      var normalY = 0;
      var planePointX = 0;
      var planePointY = 0;
      var clipPointX = 0;
      var clipPointY = 0;
      switch (manifold.type) {
      case Manifold.e_circles:
         {
            tMat = xfA.R;
            tVec = manifold.localPoint;
            var pointAX = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            var pointAY = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            tMat = xfB.R;
            tVec = manifold.points[0].localPoint;
            var pointBX = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            var pointBY = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            var dX = pointBX - pointAX;
            var dY = pointBY - pointAY;
            var d2 = dX * dX + dY * dY;
            if (d2 > Number.MIN_VALUE * Number.MIN_VALUE) {
               var d = Math.sqrt(d2);
               this.normal.x = dX / d;
               this.normal.y = dY / d;
            }
            else {
               this.normal.x = 1;
               this.normal.y = 0;
            }
            var cAX = pointAX + radiusA * this.normal.x;
            var cAY = pointAY + radiusA * this.normal.y;
            var cBX = pointBX - radiusB * this.normal.x;
            var cBY = pointBY - radiusB * this.normal.y;
            this.points[0].x = 0.5 * (cAX + cBX);
            this.points[0].y = 0.5 * (cAY + cBY);
         }
         break;
      case Manifold.e_faceA:
         {
            tMat = xfA.R;
            tVec = manifold.localPlaneNormal;
            normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            tMat = xfA.R;
            tVec = manifold.localPoint;
            planePointX = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            planePointY = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            this.normal.x = normalX;
            this.normal.y = normalY;
            for (i = 0;
            i < manifold.pointCount; i++) {
               tMat = xfB.R;
               tVec = manifold.points[i].localPoint;
               clipPointX = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
               clipPointY = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
               this.points[i].x = clipPointX + 0.5 * (radiusA - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusB) * normalX;
               this.points[i].y = clipPointY + 0.5 * (radiusA - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusB) * normalY;
            }
         }
         break;
      case Manifold.e_faceB:
         {
            tMat = xfB.R;
            tVec = manifold.localPlaneNormal;
            normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            tMat = xfB.R;
            tVec = manifold.localPoint;
            planePointX = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            planePointY = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            this.normal.x = (-normalX);
            this.normal.y = (-normalY);
            for (i = 0;
            i < manifold.pointCount; i++) {
               tMat = xfA.R;
               tVec = manifold.points[i].localPoint;
               clipPointX = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
               clipPointY = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
               this.points[i].x = clipPointX + 0.5 * (radiusB - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusA) * normalX;
               this.points[i].y = clipPointY + 0.5 * (radiusB - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusA) * normalY;
            }
         }
         break;
      }
   }
   ClipVertex.ClipVertex = function () {
      this.v = new Vec2();
      this.id = new ContactID();
   };
   ClipVertex.prototype.Set = function (other) {
      this.v.SetV(other.v);
      this.id.Set(other.id);
   }
   Features.Features = function () {};
   Object.defineProperty(Features.prototype, 'referenceEdge', {
      enumerable: false,
      configurable: true,
      get: function () {
         return this._referenceEdge;
      }
   });
   Object.defineProperty(Features.prototype, 'referenceEdge', {
      enumerable: false,
      configurable: true,
      set: function (value) {
         if (value === undefined) value = 0;
         this._referenceEdge = value;
         this._id._key = (this._id._key & 0xffffff00) | (this._referenceEdge & 0x000000ff);
      }
   });
   Object.defineProperty(Features.prototype, 'incidentEdge', {
      enumerable: false,
      configurable: true,
      get: function () {
         return this._incidentEdge;
      }
   });
   Object.defineProperty(Features.prototype, 'incidentEdge', {
      enumerable: false,
      configurable: true,
      set: function (value) {
         if (value === undefined) value = 0;
         this._incidentEdge = value;
         this._id._key = (this._id._key & 0xffff00ff) | ((this._incidentEdge << 8) & 0x0000ff00);
      }
   });
   Object.defineProperty(Features.prototype, 'incidentVertex', {
      enumerable: false,
      configurable: true,
      get: function () {
         return this._incidentVertex;
      }
   });
   Object.defineProperty(Features.prototype, 'incidentVertex', {
      enumerable: false,
      configurable: true,
      set: function (value) {
         if (value === undefined) value = 0;
         this._incidentVertex = value;
         this._id._key = (this._id._key & 0xff00ffff) | ((this._incidentVertex << 16) & 0x00ff0000);
      }
   });
   Object.defineProperty(Features.prototype, 'flip', {
      enumerable: false,
      configurable: true,
      get: function () {
         return this._flip;
      }
   });
   Object.defineProperty(Features.prototype, 'flip', {
      enumerable: false,
      configurable: true,
      set: function (value) {
         if (value === undefined) value = 0;
         this._flip = value;
         this._id._key = (this._id._key & 0x00ffffff) | ((this._flip << 24) & 0xff000000);
      }
   });
})();
(function () {
   var Color = Box2D.Common.Color,
      internal = Box2D.Common.internal,
      Settings = Box2D.Common.Settings,
      CircleShape = Box2D.Collision.Shapes.CircleShape,
      EdgeChainDef = Box2D.Collision.Shapes.EdgeChainDef,
      EdgeShape = Box2D.Collision.Shapes.EdgeShape,
      MassData = Box2D.Collision.Shapes.MassData,
      PolygonShape = Box2D.Collision.Shapes.PolygonShape,
      Shape = Box2D.Collision.Shapes.Shape,
      Mat22 = Box2D.Common.Math.Mat22,
      Mat33 = Box2D.Common.Math.Mat33,
      Math = Box2D.Common.Math.Math,
      Sweep = Box2D.Common.Math.Sweep,
      Transform = Box2D.Common.Math.Transform,
      Vec2 = Box2D.Common.Math.Vec2,
      Vec3 = Box2D.Common.Math.Vec3,
      Body = Box2D.Dynamics.Body,
      BodyDef = Box2D.Dynamics.BodyDef,
      ContactFilter = Box2D.Dynamics.ContactFilter,
      ContactImpulse = Box2D.Dynamics.ContactImpulse,
      ContactListener = Box2D.Dynamics.ContactListener,
      ContactManager = Box2D.Dynamics.ContactManager,
      DebugDraw = Box2D.Dynamics.DebugDraw,
      DestructionListener = Box2D.Dynamics.DestructionListener,
      FilterData = Box2D.Dynamics.FilterData,
      Fixture = Box2D.Dynamics.Fixture,
      FixtureDef = Box2D.Dynamics.FixtureDef,
      Island = Box2D.Dynamics.Island,
      TimeStep = Box2D.Dynamics.TimeStep,
      World = Box2D.Dynamics.World,
      AABB = Box2D.Collision.AABB,
      Bound = Box2D.Collision.Bound,
      BoundValues = Box2D.Collision.BoundValues,
      Collision = Box2D.Collision.Collision,
      ContactID = Box2D.Collision.ContactID,
      ContactPoint = Box2D.Collision.ContactPoint,
      Distance = Box2D.Collision.Distance,
      DistanceInput = Box2D.Collision.DistanceInput,
      DistanceOutput = Box2D.Collision.DistanceOutput,
      DistanceProxy = Box2D.Collision.DistanceProxy,
      DynamicTree = Box2D.Collision.DynamicTree,
      DynamicTreeBroadPhase = Box2D.Collision.DynamicTreeBroadPhase,
      DynamicTreeNode = Box2D.Collision.DynamicTreeNode,
      DynamicTreePair = Box2D.Collision.DynamicTreePair,
      Manifold = Box2D.Collision.Manifold,
      ManifoldPoint = Box2D.Collision.ManifoldPoint,
      Point = Box2D.Collision.Point,
      RayCastInput = Box2D.Collision.RayCastInput,
      RayCastOutput = Box2D.Collision.RayCastOutput,
      Segment = Box2D.Collision.Segment,
      SeparationFunction = Box2D.Collision.SeparationFunction,
      Simplex = Box2D.Collision.Simplex,
      SimplexCache = Box2D.Collision.SimplexCache,
      SimplexVertex = Box2D.Collision.SimplexVertex,
      TimeOfImpact = Box2D.Collision.TimeOfImpact,
      TOIInput = Box2D.Collision.TOIInput,
      WorldManifold = Box2D.Collision.WorldManifold,
      ClipVertex = Box2D.Collision.ClipVertex,
      Features = Box2D.Collision.Features,
      IBroadPhase = Box2D.Collision.IBroadPhase;

   Box2D.inherit(CircleShape, Box2D.Collision.Shapes.Shape);
   CircleShape.prototype.__super = Box2D.Collision.Shapes.Shape.prototype;
   CircleShape.CircleShape = function () {
      Box2D.Collision.Shapes.Shape.Shape.apply(this, arguments);
      this.p = new Vec2();
   };
   CircleShape.prototype.Copy = function () {
      var s = new CircleShape();
      s.Set(this);
      return s;
   }
   CircleShape.prototype.Set = function (other) {
      this.__super.Set.call(this, other);
      if (Box2D.is(other, CircleShape)) {
         var other2 = (other instanceof CircleShape ? other : null);
         this.p.SetV(other2.p);
      }
   }
   CircleShape.prototype.TestPoint = function (transform, p) {
      var tMat = transform.R;
      var dX = transform.position.x + (tMat.col1.x * this.p.x + tMat.col2.x * this.p.y);
      var dY = transform.position.y + (tMat.col1.y * this.p.x + tMat.col2.y * this.p.y);
      dX = p.x - dX;
      dY = p.y - dY;
      return (dX * dX + dY * dY) <= this.radius * this.radius;
   }
   CircleShape.prototype.RayCast = function (output, input, transform) {
      var tMat = transform.R;
      var positionX = transform.position.x + (tMat.col1.x * this.p.x + tMat.col2.x * this.p.y);
      var positionY = transform.position.y + (tMat.col1.y * this.p.x + tMat.col2.y * this.p.y);
      var sX = input.p1.x - positionX;
      var sY = input.p1.y - positionY;
      var b = (sX * sX + sY * sY) - this.radius * this.radius;
      var rX = input.p2.x - input.p1.x;
      var rY = input.p2.y - input.p1.y;
      var c = (sX * rX + sY * rY);
      var rr = (rX * rX + rY * rY);
      var sigma = c * c - rr * b;
      if (sigma < 0.0 || rr < Number.MIN_VALUE) {
         return false;
      }
      var a = (-(c + Math.sqrt(sigma)));
      if (0.0 <= a && a <= input.maxFraction * rr) {
         a /= rr;
         output.fraction = a;
         output.normal.x = sX + a * rX;
         output.normal.y = sY + a * rY;
         output.normal.Normalize();
         return true;
      }
      return false;
   }
   CircleShape.prototype.ComputeAABB = function (aabb, transform) {
      var tMat = transform.R;
      var pX = transform.position.x + (tMat.col1.x * this.p.x + tMat.col2.x * this.p.y);
      var pY = transform.position.y + (tMat.col1.y * this.p.x + tMat.col2.y * this.p.y);
      aabb.lowerBound.Set(pX - this.radius, pY - this.radius);
      aabb.upperBound.Set(pX + this.radius, pY + this.radius);
   }
   CircleShape.prototype.ComputeMass = function (massData, density) {
      if (density === undefined) density = 0;
      massData.mass = density * Settings._pi * this.radius * this.radius;
      massData.center.SetV(this.p);
      massData.I = massData.mass * (0.5 * this.radius * this.radius + (this.p.x * this.p.x + this.p.y * this.p.y));
   }
   CircleShape.prototype.ComputeSubmergedArea = function (normal, offset, xf, c) {
      if (offset === undefined) offset = 0;
      var p = Math.MulX(xf, this.p);
      var l = (-(Math.Dot(normal, p) - offset));
      if (l < (-this.radius) + Number.MIN_VALUE) {
         return 0;
      }
      if (l > this.radius) {
         c.SetV(p);
         return Math.PI * this.radius * this.radius;
      }
      var r2 = this.radius * this.radius;
      var l2 = l * l;
      var area = r2 * (Math.asin(l / this.radius) + Math.PI / 2) + l * Math.sqrt(r2 - l2);
      var com = (-2 / 3 * Math.pow(r2 - l2, 1.5) / area);
      c.x = p.x + normal.x * com;
      c.y = p.y + normal.y * com;
      return area;
   }
   CircleShape.prototype.GetLocalPosition = function () {
      return this.p;
   }
   CircleShape.prototype.SetLocalPosition = function (position) {
      this.p.SetV(position);
   }
   CircleShape.prototype.GetRadius = function () {
      return this.radius;
   }
   CircleShape.prototype.SetRadius = function (radius) {
      if (radius === undefined) radius = 0;
      this.radius = radius;
   }
   CircleShape.prototype.CircleShape = function (radius) {
      if (radius === undefined) radius = 0;
      this.__super.Shape.call(this);
      this.type = Shape.e_circleShape;
      this.radius = radius;
   }
   EdgeChainDef.EdgeChainDef = function () {};
   EdgeChainDef.prototype.EdgeChainDef = function () {
      this.vertexCount = 0;
      this.isALoop = true;
      this.vertices = [];
   }
   Box2D.inherit(EdgeShape, Box2D.Collision.Shapes.Shape);
   EdgeShape.prototype.__super = Box2D.Collision.Shapes.Shape.prototype;
   EdgeShape.EdgeShape = function () {
      Box2D.Collision.Shapes.Shape.Shape.apply(this, arguments);
      this.s_supportVec = new Vec2();
      this.v1 = new Vec2();
      this.v2 = new Vec2();
      this.coreV1 = new Vec2();
      this.coreV2 = new Vec2();
      this.normal = new Vec2();
      this.direction = new Vec2();
      this.cornerDir1 = new Vec2();
      this.cornerDir2 = new Vec2();
   };
   EdgeShape.prototype.TestPoint = function (transform, p) {
      return false;
   }
   EdgeShape.prototype.RayCast = function (output, input, transform) {
      var tMat;
      var rX = input.p2.x - input.p1.x;
      var rY = input.p2.y - input.p1.y;
      tMat = transform.R;
      var v1X = transform.position.x + (tMat.col1.x * this.v1.x + tMat.col2.x * this.v1.y);
      var v1Y = transform.position.y + (tMat.col1.y * this.v1.x + tMat.col2.y * this.v1.y);
      var nX = transform.position.y + (tMat.col1.y * this.v2.x + tMat.col2.y * this.v2.y) - v1Y;
      var nY = (-(transform.position.x + (tMat.col1.x * this.v2.x + tMat.col2.x * this.v2.y) - v1X));
      var k_slop = 100.0 * Number.MIN_VALUE;
      var denom = (-(rX * nX + rY * nY));
      if (denom > k_slop) {
         var bX = input.p1.x - v1X;
         var bY = input.p1.y - v1Y;
         var a = (bX * nX + bY * nY);
         if (0.0 <= a && a <= input.maxFraction * denom) {
            var mu2 = (-rX * bY) + rY * bX;
            if ((-k_slop * denom) <= mu2 && mu2 <= denom * (1.0 + k_slop)) {
               a /= denom;
               output.fraction = a;
               var nLen = Math.sqrt(nX * nX + nY * nY);
               output.normal.x = nX / nLen;
               output.normal.y = nY / nLen;
               return true;
            }
         }
      }
      return false;
   }
   EdgeShape.prototype.ComputeAABB = function (aabb, transform) {
      var tMat = transform.R;
      var v1X = transform.position.x + (tMat.col1.x * this.v1.x + tMat.col2.x * this.v1.y);
      var v1Y = transform.position.y + (tMat.col1.y * this.v1.x + tMat.col2.y * this.v1.y);
      var v2X = transform.position.x + (tMat.col1.x * this.v2.x + tMat.col2.x * this.v2.y);
      var v2Y = transform.position.y + (tMat.col1.y * this.v2.x + tMat.col2.y * this.v2.y);
      if (v1X < v2X) {
         aabb.lowerBound.x = v1X;
         aabb.upperBound.x = v2X;
      }
      else {
         aabb.lowerBound.x = v2X;
         aabb.upperBound.x = v1X;
      }
      if (v1Y < v2Y) {
         aabb.lowerBound.y = v1Y;
         aabb.upperBound.y = v2Y;
      }
      else {
         aabb.lowerBound.y = v2Y;
         aabb.upperBound.y = v1Y;
      }
   }
   EdgeShape.prototype.ComputeMass = function (massData, density) {
      if (density === undefined) density = 0;
      massData.mass = 0;
      massData.center.SetV(this.v1);
      massData.I = 0;
   }
   EdgeShape.prototype.ComputeSubmergedArea = function (normal, offset, xf, c) {
      if (offset === undefined) offset = 0;
      var v0 = new Vec2(normal.x * offset, normal.y * offset);
      var v1 = Math.MulX(xf, this.v1);
      var v2 = Math.MulX(xf, this.v2);
      var d1 = Math.Dot(normal, v1) - offset;
      var d2 = Math.Dot(normal, v2) - offset;
      if (d1 > 0) {
         if (d2 > 0) {
            return 0;
         }
         else {
            v1.x = (-d2 / (d1 - d2) * v1.x) + d1 / (d1 - d2) * v2.x;
            v1.y = (-d2 / (d1 - d2) * v1.y) + d1 / (d1 - d2) * v2.y;
         }
      }
      else {
         if (d2 > 0) {
            v2.x = (-d2 / (d1 - d2) * v1.x) + d1 / (d1 - d2) * v2.x;
            v2.y = (-d2 / (d1 - d2) * v1.y) + d1 / (d1 - d2) * v2.y;
         }
         else {}
      }
      c.x = (v0.x + v1.x + v2.x) / 3;
      c.y = (v0.y + v1.y + v2.y) / 3;
      return 0.5 * ((v1.x - v0.x) * (v2.y - v0.y) - (v1.y - v0.y) * (v2.x - v0.x));
   }
   EdgeShape.prototype.GetLength = function () {
      return this.length;
   }
   EdgeShape.prototype.GetVertex1 = function () {
      return this.v1;
   }
   EdgeShape.prototype.GetVertex2 = function () {
      return this.v2;
   }
   EdgeShape.prototype.GetCoreVertex1 = function () {
      return this.coreV1;
   }
   EdgeShape.prototype.GetCoreVertex2 = function () {
      return this.coreV2;
   }
   EdgeShape.prototype.GetNormalVector = function () {
      return this.normal;
   }
   EdgeShape.prototype.GetDirectionVector = function () {
      return this.direction;
   }
   EdgeShape.prototype.GetCorner1Vector = function () {
      return this.cornerDir1;
   }
   EdgeShape.prototype.GetCorner2Vector = function () {
      return this.cornerDir2;
   }
   EdgeShape.prototype.Corner1IsConvex = function () {
      return this.cornerConvex1;
   }
   EdgeShape.prototype.Corner2IsConvex = function () {
      return this.cornerConvex2;
   }
   EdgeShape.prototype.GetFirstVertex = function (xf) {
      var tMat = xf.R;
      return new Vec2(xf.position.x + (tMat.col1.x * this.coreV1.x + tMat.col2.x * this.coreV1.y), xf.position.y + (tMat.col1.y * this.coreV1.x + tMat.col2.y * this.coreV1.y));
   }
   EdgeShape.prototype.GetNextEdge = function () {
      return this.nextEdge;
   }
   EdgeShape.prototype.GetPrevEdge = function () {
      return this.prevEdge;
   }
   EdgeShape.prototype.Support = function (xf, dX, dY) {
      if (dX === undefined) dX = 0;
      if (dY === undefined) dY = 0;
      var tMat = xf.R;
      var v1X = xf.position.x + (tMat.col1.x * this.coreV1.x + tMat.col2.x * this.coreV1.y);
      var v1Y = xf.position.y + (tMat.col1.y * this.coreV1.x + tMat.col2.y * this.coreV1.y);
      var v2X = xf.position.x + (tMat.col1.x * this.coreV2.x + tMat.col2.x * this.coreV2.y);
      var v2Y = xf.position.y + (tMat.col1.y * this.coreV2.x + tMat.col2.y * this.coreV2.y);
      if ((v1X * dX + v1Y * dY) > (v2X * dX + v2Y * dY)) {
         this.s_supportVec.x = v1X;
         this.s_supportVec.y = v1Y;
      }
      else {
         this.s_supportVec.x = v2X;
         this.s_supportVec.y = v2Y;
      }
      return this.s_supportVec;
   }
   EdgeShape.prototype.EdgeShape = function (v1, v2) {
      this.__super.Shape.call(this);
      this.type = Shape.e_edgeShape;
      this.prevEdge = null;
      this.nextEdge = null;
      this.v1 = v1;
      this.v2 = v2;
      this.direction.Set(this.v2.x - this.v1.x, this.v2.y - this.v1.y);
      this.length = this.direction.Normalize();
      this.normal.Set(this.direction.y, (-this.direction.x));
      this.coreV1.Set((-Settings._toiSlop * (this.normal.x - this.direction.x)) + this.v1.x, (-Settings._toiSlop * (this.normal.y - this.direction.y)) + this.v1.y);
      this.coreV2.Set((-Settings._toiSlop * (this.normal.x + this.direction.x)) + this.v2.x, (-Settings._toiSlop * (this.normal.y + this.direction.y)) + this.v2.y);
      this.cornerDir1 = this.normal;
      this.cornerDir2.Set((-this.normal.x), (-this.normal.y));
   }
   EdgeShape.prototype.SetPrevEdge = function (edge, core, cornerDir, convex) {
      this.prevEdge = edge;
      this.coreV1 = core;
      this.cornerDir1 = cornerDir;
      this.cornerConvex1 = convex;
   }
   EdgeShape.prototype.SetNextEdge = function (edge, core, cornerDir, convex) {
      this.nextEdge = edge;
      this.coreV2 = core;
      this.cornerDir2 = cornerDir;
      this.cornerConvex2 = convex;
   }
   MassData.MassData = function () {
      this.mass = 0.0;
      this.center = new Vec2(0, 0);
      this.I = 0.0;
   };
   Box2D.inherit(PolygonShape, Box2D.Collision.Shapes.Shape);
   PolygonShape.prototype.__super = Box2D.Collision.Shapes.Shape.prototype;
   PolygonShape.PolygonShape = function () {
      Box2D.Collision.Shapes.Shape.Shape.apply(this, arguments);
   };
   PolygonShape.prototype.Copy = function () {
      var s = new PolygonShape();
      s.Set(this);
      return s;
   }
   PolygonShape.prototype.Set = function (other) {
      this.__super.Set.call(this, other);
      if (Box2D.is(other, PolygonShape)) {
         var other2 = (other instanceof PolygonShape ? other : null);
         this.centroid.SetV(other2.centroid);
         this.vertexCount = other2.vertexCount;
         this.Reserve(this.vertexCount);
         for (var i = 0; i < this.vertexCount; i++) {
            this.vertices[i].SetV(other2.vertices[i]);
            this.normals[i].SetV(other2.normals[i]);
         }
      }
   }
   PolygonShape.prototype.SetAsArray = function (vertices, vertexCount) {
      if (vertexCount === undefined) vertexCount = 0;
      var v = new Vector();
      var i = 0,
         tVec;
      for (i = 0;
      i < vertices.length; ++i) {
         tVec = vertices[i];
         v.push(tVec);
      }
      this.SetAsVector(v, vertexCount);
   }
   PolygonShape.AsArray = function (vertices, vertexCount) {
      if (vertexCount === undefined) vertexCount = 0;
      var polygonShape = new PolygonShape();
      polygonShape.SetAsArray(vertices, vertexCount);
      return polygonShape;
   }
   PolygonShape.prototype.SetAsVector = function (vertices, vertexCount) {
      if (vertexCount === undefined) vertexCount = 0;
      if (vertexCount == 0) vertexCount = vertices.length;
      Settings.Assert(2 <= vertexCount);
      this.vertexCount = vertexCount;
      this.Reserve(vertexCount);
      var i = 0;
      for (i = 0;
      i < this.vertexCount; i++) {
         this.vertices[i].SetV(vertices[i]);
      }
      for (i = 0;
      i < this.vertexCount; ++i) {
         var i1 = parseInt(i);
         var i2 = parseInt(i + 1 < this.vertexCount ? i + 1 : 0);
         var edge = Math.SubtractVV(this.vertices[i2], this.vertices[i1]);
         Settings.Assert(edge.LengthSquared() > Number.MIN_VALUE);
         this.normals[i].SetV(Math.CrossVF(edge, 1.0));
         this.normals[i].Normalize();
      }
      this.centroid = PolygonShape.ComputeCentroid(this.vertices, this.vertexCount);
   }
   PolygonShape.AsVector = function (vertices, vertexCount) {
      if (vertexCount === undefined) vertexCount = 0;
      var polygonShape = new PolygonShape();
      polygonShape.SetAsVector(vertices, vertexCount);
      return polygonShape;
   }
   PolygonShape.prototype.SetAsBox = function (hx, hy) {
      if (hx === undefined) hx = 0;
      if (hy === undefined) hy = 0;
      this.vertexCount = 4;
      this.Reserve(4);
      this.vertices[0].Set((-hx), (-hy));
      this.vertices[1].Set(hx, (-hy));
      this.vertices[2].Set(hx, hy);
      this.vertices[3].Set((-hx), hy);
      this.normals[0].Set(0.0, (-1.0));
      this.normals[1].Set(1.0, 0.0);
      this.normals[2].Set(0.0, 1.0);
      this.normals[3].Set((-1.0), 0.0);
      this.centroid.SetZero();
   }
   PolygonShape.AsBox = function (hx, hy) {
      if (hx === undefined) hx = 0;
      if (hy === undefined) hy = 0;
      var polygonShape = new PolygonShape();
      polygonShape.SetAsBox(hx, hy);
      return polygonShape;
   }
   PolygonShape.prototype.SetAsOrientedBox = function (hx, hy, center, angle) {
      if (hx === undefined) hx = 0;
      if (hy === undefined) hy = 0;
      if (center === undefined) center = null;
      if (angle === undefined) angle = 0.0;
      this.vertexCount = 4;
      this.Reserve(4);
      this.vertices[0].Set((-hx), (-hy));
      this.vertices[1].Set(hx, (-hy));
      this.vertices[2].Set(hx, hy);
      this.vertices[3].Set((-hx), hy);
      this.normals[0].Set(0.0, (-1.0));
      this.normals[1].Set(1.0, 0.0);
      this.normals[2].Set(0.0, 1.0);
      this.normals[3].Set((-1.0), 0.0);
      this.centroid = center;
      var xf = new Transform();
      xf.position = center;
      xf.R.Set(angle);
      for (var i = 0; i < this.vertexCount; ++i) {
         this.vertices[i] = Math.MulX(xf, this.vertices[i]);
         this.normals[i] = Math.MulMV(xf.R, this.normals[i]);
      }
   }
   PolygonShape.AsOrientedBox = function (hx, hy, center, angle) {
      if (hx === undefined) hx = 0;
      if (hy === undefined) hy = 0;
      if (center === undefined) center = null;
      if (angle === undefined) angle = 0.0;
      var polygonShape = new PolygonShape();
      polygonShape.SetAsOrientedBox(hx, hy, center, angle);
      return polygonShape;
   }
   PolygonShape.prototype.SetAsEdge = function (v1, v2) {
      this.vertexCount = 2;
      this.Reserve(2);
      this.vertices[0].SetV(v1);
      this.vertices[1].SetV(v2);
      this.centroid.x = 0.5 * (v1.x + v2.x);
      this.centroid.y = 0.5 * (v1.y + v2.y);
      this.normals[0] = Math.CrossVF(Math.SubtractVV(v2, v1), 1.0);
      this.normals[0].Normalize();
      this.normals[1].x = (-this.normals[0].x);
      this.normals[1].y = (-this.normals[0].y);
   }
   PolygonShape.AsEdge = function (v1, v2) {
      var polygonShape = new PolygonShape();
      polygonShape.SetAsEdge(v1, v2);
      return polygonShape;
   }
   PolygonShape.prototype.TestPoint = function (xf, p) {
      var tVec;
      var tMat = xf.R;
      var tX = p.x - xf.position.x;
      var tY = p.y - xf.position.y;
      var pLocalX = (tX * tMat.col1.x + tY * tMat.col1.y);
      var pLocalY = (tX * tMat.col2.x + tY * tMat.col2.y);
      for (var i = 0; i < this.vertexCount; ++i) {
         tVec = this.vertices[i];
         tX = pLocalX - tVec.x;
         tY = pLocalY - tVec.y;
         tVec = this.normals[i];
         var dot = (tVec.x * tX + tVec.y * tY);
         if (dot > 0.0) {
            return false;
         }
      }
      return true;
   }
   PolygonShape.prototype.RayCast = function (output, input, transform) {
      var lower = 0.0;
      var upper = input.maxFraction;
      var tX = 0;
      var tY = 0;
      var tMat;
      var tVec;
      tX = input.p1.x - transform.position.x;
      tY = input.p1.y - transform.position.y;
      tMat = transform.R;
      var p1X = (tX * tMat.col1.x + tY * tMat.col1.y);
      var p1Y = (tX * tMat.col2.x + tY * tMat.col2.y);
      tX = input.p2.x - transform.position.x;
      tY = input.p2.y - transform.position.y;
      tMat = transform.R;
      var p2X = (tX * tMat.col1.x + tY * tMat.col1.y);
      var p2Y = (tX * tMat.col2.x + tY * tMat.col2.y);
      var dX = p2X - p1X;
      var dY = p2Y - p1Y;
      var index = parseInt((-1));
      for (var i = 0; i < this.vertexCount; ++i) {
         tVec = this.vertices[i];
         tX = tVec.x - p1X;
         tY = tVec.y - p1Y;
         tVec = this.normals[i];
         var numerator = (tVec.x * tX + tVec.y * tY);
         var denominator = (tVec.x * dX + tVec.y * dY);
         if (denominator == 0.0) {
            if (numerator < 0.0) {
               return false;
            }
         }
         else {
            if (denominator < 0.0 && numerator < lower * denominator) {
               lower = numerator / denominator;
               index = i;
            }
            else if (denominator > 0.0 && numerator < upper * denominator) {
               upper = numerator / denominator;
            }
         }
         if (upper < lower - Number.MIN_VALUE) {
            return false;
         }
      }
      if (index >= 0) {
         output.fraction = lower;
         tMat = transform.R;
         tVec = this.normals[index];
         output.normal.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
         output.normal.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
         return true;
      }
      return false;
   }
   PolygonShape.prototype.ComputeAABB = function (aabb, xf) {
      var tMat = xf.R;
      var tVec = this.vertices[0];
      var lowerX = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      var lowerY = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      var upperX = lowerX;
      var upperY = lowerY;
      for (var i = 1; i < this.vertexCount; ++i) {
         tVec = this.vertices[i];
         var vX = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
         var vY = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
         lowerX = lowerX < vX ? lowerX : vX;
         lowerY = lowerY < vY ? lowerY : vY;
         upperX = upperX > vX ? upperX : vX;
         upperY = upperY > vY ? upperY : vY;
      }
      aabb.lowerBound.x = lowerX - this.radius;
      aabb.lowerBound.y = lowerY - this.radius;
      aabb.upperBound.x = upperX + this.radius;
      aabb.upperBound.y = upperY + this.radius;
   }
   PolygonShape.prototype.ComputeMass = function (massData, density) {
      if (density === undefined) density = 0;
      if (this.vertexCount == 2) {
         massData.center.x = 0.5 * (this.vertices[0].x + this.vertices[1].x);
         massData.center.y = 0.5 * (this.vertices[0].y + this.vertices[1].y);
         massData.mass = 0.0;
         massData.I = 0.0;
         return;
      }
      var centerX = 0.0;
      var centerY = 0.0;
      var area = 0.0;
      var I = 0.0;
      var p1X = 0.0;
      var p1Y = 0.0;
      var k_inv3 = 1.0 / 3.0;
      for (var i = 0; i < this.vertexCount; ++i) {
         var p2 = this.vertices[i];
         var p3 = i + 1 < this.vertexCount ? this.vertices[parseInt(i + 1)] : this.vertices[0];
         var e1X = p2.x - p1X;
         var e1Y = p2.y - p1Y;
         var e2X = p3.x - p1X;
         var e2Y = p3.y - p1Y;
         var D = e1X * e2Y - e1Y * e2X;
         var triangleArea = 0.5 * D;area += triangleArea;
         centerX += triangleArea * k_inv3 * (p1X + p2.x + p3.x);
         centerY += triangleArea * k_inv3 * (p1Y + p2.y + p3.y);
         var px = p1X;
         var py = p1Y;
         var ex1 = e1X;
         var ey1 = e1Y;
         var ex2 = e2X;
         var ey2 = e2Y;
         var intx2 = k_inv3 * (0.25 * (ex1 * ex1 + ex2 * ex1 + ex2 * ex2) + (px * ex1 + px * ex2)) + 0.5 * px * px;
         var inty2 = k_inv3 * (0.25 * (ey1 * ey1 + ey2 * ey1 + ey2 * ey2) + (py * ey1 + py * ey2)) + 0.5 * py * py;I += D * (intx2 + inty2);
      }
      massData.mass = density * area;
      centerX *= 1.0 / area;
      centerY *= 1.0 / area;
      massData.center.Set(centerX, centerY);
      massData.I = density * I;
   }
   PolygonShape.prototype.ComputeSubmergedArea = function (normal, offset, xf, c) {
      if (offset === undefined) offset = 0;
      var normalL = Math.MulTMV(xf.R, normal);
      var offsetL = offset - Math.Dot(normal, xf.position);
      var depths = new Vector_a2j_Number();
      var diveCount = 0;
      var intoIndex = parseInt((-1));
      var outoIndex = parseInt((-1));
      var lastSubmerged = false;
      var i = 0;
      for (i = 0;
      i < this.vertexCount; ++i) {
         depths[i] = Math.Dot(normalL, this.vertices[i]) - offsetL;
         var isSubmerged = depths[i] < (-Number.MIN_VALUE);
         if (i > 0) {
            if (isSubmerged) {
               if (!lastSubmerged) {
                  intoIndex = i - 1;
                  diveCount++;
               }
            }
            else {
               if (lastSubmerged) {
                  outoIndex = i - 1;
                  diveCount++;
               }
            }
         }
         lastSubmerged = isSubmerged;
      }
      switch (diveCount) {
      case 0:
         if (lastSubmerged) {
            var md = new MassData();
            this.ComputeMass(md, 1);
            c.SetV(Math.MulX(xf, md.center));
            return md.mass;
         }
         else {
            return 0;
         }
         break;
      case 1:
         if (intoIndex == (-1)) {
            intoIndex = this.vertexCount - 1;
         }
         else {
            outoIndex = this.vertexCount - 1;
         }
         break;
      }
      var intoIndex2 = parseInt((intoIndex + 1) % this.vertexCount);
      var outoIndex2 = parseInt((outoIndex + 1) % this.vertexCount);
      var intoLamdda = (0 - depths[intoIndex]) / (depths[intoIndex2] - depths[intoIndex]);
      var outoLamdda = (0 - depths[outoIndex]) / (depths[outoIndex2] - depths[outoIndex]);
      var intoVec = new Vec2(this.vertices[intoIndex].x * (1 - intoLamdda) + this.vertices[intoIndex2].x * intoLamdda, this.vertices[intoIndex].y * (1 - intoLamdda) + this.vertices[intoIndex2].y * intoLamdda);
      var outoVec = new Vec2(this.vertices[outoIndex].x * (1 - outoLamdda) + this.vertices[outoIndex2].x * outoLamdda, this.vertices[outoIndex].y * (1 - outoLamdda) + this.vertices[outoIndex2].y * outoLamdda);
      var area = 0;
      var center = new Vec2();
      var p2 = this.vertices[intoIndex2];
      var p3;
      i = intoIndex2;
      while (i != outoIndex2) {
         i = (i + 1) % this.vertexCount;
         if (i == outoIndex2) p3 = outoVec;
         else p3 = this.vertices[i];
         var triangleArea = 0.5 * ((p2.x - intoVec.x) * (p3.y - intoVec.y) - (p2.y - intoVec.y) * (p3.x - intoVec.x));
         area += triangleArea;
         center.x += triangleArea * (intoVec.x + p2.x + p3.x) / 3;
         center.y += triangleArea * (intoVec.y + p2.y + p3.y) / 3;
         p2 = p3;
      }
      center.Multiply(1 / area);
      c.SetV(Math.MulX(xf, center));
      return area;
   }
   PolygonShape.prototype.GetVertexCount = function () {
      return this.vertexCount;
   }
   PolygonShape.prototype.GetVertices = function () {
      return this.vertices;
   }
   PolygonShape.prototype.GetNormals = function () {
      return this.normals;
   }
   PolygonShape.prototype.GetSupport = function (d) {
      var bestIndex = 0;
      var bestValue = this.vertices[0].x * d.x + this.vertices[0].y * d.y;
      for (var i = 1; i < this.vertexCount; ++i) {
         var value = this.vertices[i].x * d.x + this.vertices[i].y * d.y;
         if (value > bestValue) {
            bestIndex = i;
            bestValue = value;
         }
      }
      return bestIndex;
   }
   PolygonShape.prototype.GetSupportVertex = function (d) {
      var bestIndex = 0;
      var bestValue = this.vertices[0].x * d.x + this.vertices[0].y * d.y;
      for (var i = 1; i < this.vertexCount; ++i) {
         var value = this.vertices[i].x * d.x + this.vertices[i].y * d.y;
         if (value > bestValue) {
            bestIndex = i;
            bestValue = value;
         }
      }
      return this.vertices[bestIndex];
   }
   PolygonShape.prototype.Validate = function () {
      return false;
   }
   PolygonShape.prototype.PolygonShape = function () {
      this.__super.Shape.call(this);
      this.type = Shape.e_polygonShape;
      this.centroid = new Vec2();
      this.vertices = new Vector();
      this.normals = new Vector();
   }
   PolygonShape.prototype.Reserve = function (count) {
      if (count === undefined) count = 0;
      for (var i = parseInt(this.vertices.length); i < count; i++) {
         this.vertices[i] = new Vec2();
         this.normals[i] = new Vec2();
      }
   }
   PolygonShape.ComputeCentroid = function (vs, count) {
      if (count === undefined) count = 0;
      var c = new Vec2();
      var area = 0.0;
      var p1X = 0.0;
      var p1Y = 0.0;
      var inv3 = 1.0 / 3.0;
      for (var i = 0; i < count; ++i) {
         var p2 = vs[i];
         var p3 = i + 1 < count ? vs[parseInt(i + 1)] : vs[0];
         var e1X = p2.x - p1X;
         var e1Y = p2.y - p1Y;
         var e2X = p3.x - p1X;
         var e2Y = p3.y - p1Y;
         var D = (e1X * e2Y - e1Y * e2X);
         var triangleArea = 0.5 * D;area += triangleArea;
         c.x += triangleArea * inv3 * (p1X + p2.x + p3.x);
         c.y += triangleArea * inv3 * (p1Y + p2.y + p3.y);
      }
      c.x *= 1.0 / area;
      c.y *= 1.0 / area;
      return c;
   }
   PolygonShape.ComputeOBB = function (obb, vs, count) {
      if (count === undefined) count = 0;
      var i = 0;
      var p = new Vector(count + 1);
      for (i = 0;
      i < count; ++i) {
         p[i] = vs[i];
      }
      p[count] = p[0];
      var minArea = Number.MAX_VALUE;
      for (i = 1;
      i <= count; ++i) {
         var root = p[parseInt(i - 1)];
         var uxX = p[i].x - root.x;
         var uxY = p[i].y - root.y;
         var length = Math.sqrt(uxX * uxX + uxY * uxY);
         uxX /= length;
         uxY /= length;
         var uyX = (-uxY);
         var uyY = uxX;
         var lowerX = Number.MAX_VALUE;
         var lowerY = Number.MAX_VALUE;
         var upperX = (-Number.MAX_VALUE);
         var upperY = (-Number.MAX_VALUE);
         for (var j = 0; j < count; ++j) {
            var dX = p[j].x - root.x;
            var dY = p[j].y - root.y;
            var rX = (uxX * dX + uxY * dY);
            var rY = (uyX * dX + uyY * dY);
            if (rX < lowerX) lowerX = rX;
            if (rY < lowerY) lowerY = rY;
            if (rX > upperX) upperX = rX;
            if (rY > upperY) upperY = rY;
         }
         var area = (upperX - lowerX) * (upperY - lowerY);
         if (area < 0.95 * minArea) {
            minArea = area;
            obb.R.col1.x = uxX;
            obb.R.col1.y = uxY;
            obb.R.col2.x = uyX;
            obb.R.col2.y = uyY;
            var centerX = 0.5 * (lowerX + upperX);
            var centerY = 0.5 * (lowerY + upperY);
            var tMat = obb.R;
            obb.center.x = root.x + (tMat.col1.x * centerX + tMat.col2.x * centerY);
            obb.center.y = root.y + (tMat.col1.y * centerX + tMat.col2.y * centerY);
            obb.extents.x = 0.5 * (upperX - lowerX);
            obb.extents.y = 0.5 * (upperY - lowerY);
         }
      }
   }
   Box2D.postDefs.push(function () {
      Box2D.Collision.Shapes.PolygonShape.s_mat = new Mat22();
   });
   Shape.Shape = function () {};
   Shape.prototype.Copy = function () {
      return null;
   }
   Shape.prototype.Set = function (other) {
      this.radius = other.radius;
   }
   Shape.prototype.GetType = function () {
      return this.type;
   }
   Shape.prototype.TestPoint = function (xf, p) {
      return false;
   }
   Shape.prototype.RayCast = function (output, input, transform) {
      return false;
   }
   Shape.prototype.ComputeAABB = function (aabb, xf) {}
   Shape.prototype.ComputeMass = function (massData, density) {
      if (density === undefined) density = 0;
   }
   Shape.prototype.ComputeSubmergedArea = function (normal, offset, xf, c) {
      if (offset === undefined) offset = 0;
      return 0;
   }
   Shape.TestOverlap = function (shape1, transform1, shape2, transform2) {
      var input = new DistanceInput();
      input.proxyA = new DistanceProxy();
      input.proxyA.Set(shape1);
      input.proxyB = new DistanceProxy();
      input.proxyB.Set(shape2);
      input.transformA = transform1;
      input.transformB = transform2;
      input.useRadii = true;
      var simplexCache = new SimplexCache();
      simplexCache.count = 0;
      var output = new DistanceOutput();
      Distance.Distance(output, simplexCache, input);
      return output.distance < 10.0 * Number.MIN_VALUE;
   }
   Shape.prototype.Shape = function () {
      this.type = Shape.e_unknownShape;
      this.radius = Settings._linearSlop;
   }
   Box2D.postDefs.push(function () {
      Box2D.Collision.Shapes.Shape.e_unknownShape = parseInt((-1));
      Box2D.Collision.Shapes.Shape.e_circleShape = 0;
      Box2D.Collision.Shapes.Shape.e_polygonShape = 1;
      Box2D.Collision.Shapes.Shape.e_edgeShape = 2;
      Box2D.Collision.Shapes.Shape.e_shapeTypeCount = 3;
      Box2D.Collision.Shapes.Shape.e_hitCollide = 1;
      Box2D.Collision.Shapes.Shape.e_missCollide = 0;
      Box2D.Collision.Shapes.Shape.e_startsInsideCollide = parseInt((-1));
   });
})();
(function () {
   var Color = Box2D.Common.Color,
      internal = Box2D.Common.internal,
      Settings = Box2D.Common.Settings,
      Mat22 = Box2D.Common.Math.Mat22,
      Mat33 = Box2D.Common.Math.Mat33,
      Math = Box2D.Common.Math.Math,
      Sweep = Box2D.Common.Math.Sweep,
      Transform = Box2D.Common.Math.Transform,
      Vec2 = Box2D.Common.Math.Vec2,
      Vec3 = Box2D.Common.Math.Vec3;

   Color.Color = function () {
      this._r = 0;
      this._g = 0;
      this._b = 0;
   };
   Color.prototype.Color = function (rr, gg, bb) {
      if (rr === undefined) rr = 0;
      if (gg === undefined) gg = 0;
      if (bb === undefined) bb = 0;
      this._r = Box2D.parseUInt(255 * Math.Clamp(rr, 0.0, 1.0));
      this._g = Box2D.parseUInt(255 * Math.Clamp(gg, 0.0, 1.0));
      this._b = Box2D.parseUInt(255 * Math.Clamp(bb, 0.0, 1.0));
   }
   Color.prototype.Set = function (rr, gg, bb) {
      if (rr === undefined) rr = 0;
      if (gg === undefined) gg = 0;
      if (bb === undefined) bb = 0;
      this._r = Box2D.parseUInt(255 * Math.Clamp(rr, 0.0, 1.0));
      this._g = Box2D.parseUInt(255 * Math.Clamp(gg, 0.0, 1.0));
      this._b = Box2D.parseUInt(255 * Math.Clamp(bb, 0.0, 1.0));
   }
   Object.defineProperty(Color.prototype, 'r', {
      enumerable: false,
      configurable: true,
      set: function (rr) {
         if (rr === undefined) rr = 0;
         this._r = Box2D.parseUInt(255 * Math.Clamp(rr, 0.0, 1.0));
      }
   });
   Object.defineProperty(Color.prototype, 'g', {
      enumerable: false,
      configurable: true,
      set: function (gg) {
         if (gg === undefined) gg = 0;
         this._g = Box2D.parseUInt(255 * Math.Clamp(gg, 0.0, 1.0));
      }
   });
   Object.defineProperty(Color.prototype, 'b', {
      enumerable: false,
      configurable: true,
      set: function (bb) {
         if (bb === undefined) bb = 0;
         this._b = Box2D.parseUInt(255 * Math.Clamp(bb, 0.0, 1.0));
      }
   });
   Object.defineProperty(Color.prototype, 'color', {
      enumerable: false,
      configurable: true,
      get: function () {
         return (this._r << 16) | (this._g << 8) | (this._b);
      }
   });
   Settings.Settings = function () {};
   Settings.MixFriction = function (friction1, friction2) {
      if (friction1 === undefined) friction1 = 0;
      if (friction2 === undefined) friction2 = 0;
      return Math.sqrt(friction1 * friction2);
   }
   Settings.MixRestitution = function (restitution1, restitution2) {
      if (restitution1 === undefined) restitution1 = 0;
      if (restitution2 === undefined) restitution2 = 0;
      return restitution1 > restitution2 ? restitution1 : restitution2;
   }
   Settings.Assert = function (a) {
      if (!a) {
         throw "Assertion Failed";
      }
   }
   Box2D.postDefs.push(function () {
      Box2D.Common.Settings.VERSION = "2.1alpha";
      Box2D.Common.Settings.USHRT_MAX = 0x0000ffff;
      Box2D.Common.Settings._pi = Math.PI;
      Box2D.Common.Settings._maxManifoldPoints = 2;
      Box2D.Common.Settings._aabbExtension = 0.1;
      Box2D.Common.Settings._aabbMultiplier = 2.0;
      Box2D.Common.Settings._polygonRadius = 2.0 * Settings._linearSlop;
      Box2D.Common.Settings._linearSlop = 0.005;
      Box2D.Common.Settings._angularSlop = 2.0 / 180.0 * Settings._pi;
      Box2D.Common.Settings._toiSlop = 8.0 * Settings._linearSlop;
      Box2D.Common.Settings._maxTOIContactsPerIsland = 32;
      Box2D.Common.Settings._maxTOIJointsPerIsland = 32;
      Box2D.Common.Settings._velocityThreshold = 1.0;
      Box2D.Common.Settings._maxLinearCorrection = 0.2;
      Box2D.Common.Settings._maxAngularCorrection = 8.0 / 180.0 * Settings._pi;
      Box2D.Common.Settings._maxTranslation = 2.0;
      Box2D.Common.Settings._maxTranslationSquared = Settings._maxTranslation * Settings._maxTranslation;
      Box2D.Common.Settings._maxRotation = 0.5 * Settings._pi;
      Box2D.Common.Settings._maxRotationSquared = Settings._maxRotation * Settings._maxRotation;
      Box2D.Common.Settings._contactBaumgarte = 0.2;
      Box2D.Common.Settings._timeToSleep = 0.5;
      Box2D.Common.Settings._linearSleepTolerance = 0.01;
      Box2D.Common.Settings._angularSleepTolerance = 2.0 / 180.0 * Settings._pi;
   });
})();
(function () {
   var AABB = Box2D.Collision.AABB,
      Color = Box2D.Common.Color,
      internal = Box2D.Common.internal,
      Settings = Box2D.Common.Settings,
      Mat22 = Box2D.Common.Math.Mat22,
      Mat33 = Box2D.Common.Math.Mat33,
      Math = Box2D.Common.Math.Math,
      Sweep = Box2D.Common.Math.Sweep,
      Transform = Box2D.Common.Math.Transform,
      Vec2 = Box2D.Common.Math.Vec2,
      Vec3 = Box2D.Common.Math.Vec3;

   Mat22.Mat22 = function () {
      this.col1 = new Vec2();
      this.col2 = new Vec2();
   };
   Mat22.prototype.Mat22 = function () {
      this.SetIdentity();
   }
   Mat22.FromAngle = function (angle) {
      if (angle === undefined) angle = 0;
      var mat = new Mat22();
      mat.Set(angle);
      return mat;
   }
   Mat22.FromVV = function (c1, c2) {
      var mat = new Mat22();
      mat.SetVV(c1, c2);
      return mat;
   }
   Mat22.prototype.Set = function (angle) {
      if (angle === undefined) angle = 0;
      var c = Math.cos(angle);
      var s = Math.sin(angle);
      this.col1.x = c;
      this.col2.x = (-s);
      this.col1.y = s;
      this.col2.y = c;
   }
   Mat22.prototype.SetVV = function (c1, c2) {
      this.col1.SetV(c1);
      this.col2.SetV(c2);
   }
   Mat22.prototype.Copy = function () {
      var mat = new Mat22();
      mat.SetM(this);
      return mat;
   }
   Mat22.prototype.SetM = function (m) {
      this.col1.SetV(m.col1);
      this.col2.SetV(m.col2);
   }
   Mat22.prototype.AddM = function (m) {
      this.col1.x += m.col1.x;
      this.col1.y += m.col1.y;
      this.col2.x += m.col2.x;
      this.col2.y += m.col2.y;
   }
   Mat22.prototype.SetIdentity = function () {
      this.col1.x = 1.0;
      this.col2.x = 0.0;
      this.col1.y = 0.0;
      this.col2.y = 1.0;
   }
   Mat22.prototype.SetZero = function () {
      this.col1.x = 0.0;
      this.col2.x = 0.0;
      this.col1.y = 0.0;
      this.col2.y = 0.0;
   }
   Mat22.prototype.GetAngle = function () {
      return Math.atan2(this.col1.y, this.col1.x);
   }
   Mat22.prototype.GetInverse = function (out) {
      var a = this.col1.x;
      var b = this.col2.x;
      var c = this.col1.y;
      var d = this.col2.y;
      var det = a * d - b * c;
      if (det != 0.0) {
         det = 1.0 / det;
      }
      out.col1.x = det * d;
      out.col2.x = (-det * b);
      out.col1.y = (-det * c);
      out.col2.y = det * a;
      return out;
   }
   Mat22.prototype.Solve = function (out, bX, bY) {
      if (bX === undefined) bX = 0;
      if (bY === undefined) bY = 0;
      var a11 = this.col1.x;
      var a12 = this.col2.x;
      var a21 = this.col1.y;
      var a22 = this.col2.y;
      var det = a11 * a22 - a12 * a21;
      if (det != 0.0) {
         det = 1.0 / det;
      }
      out.x = det * (a22 * bX - a12 * bY);
      out.y = det * (a11 * bY - a21 * bX);
      return out;
   }
   Mat22.prototype.Abs = function () {
      this.col1.Abs();
      this.col2.Abs();
   }
   Mat33.Mat33 = function () {
      this.col1 = new Vec3();
      this.col2 = new Vec3();
      this.col3 = new Vec3();
   };
   Mat33.prototype.Mat33 = function (c1, c2, c3) {
      if (c1 === undefined) c1 = null;
      if (c2 === undefined) c2 = null;
      if (c3 === undefined) c3 = null;
      if (!c1 && !c2 && !c3) {
         this.col1.SetZero();
         this.col2.SetZero();
         this.col3.SetZero();
      }
      else {
         this.col1.SetV(c1);
         this.col2.SetV(c2);
         this.col3.SetV(c3);
      }
   }
   Mat33.prototype.SetVVV = function (c1, c2, c3) {
      this.col1.SetV(c1);
      this.col2.SetV(c2);
      this.col3.SetV(c3);
   }
   Mat33.prototype.Copy = function () {
      return new Mat33(this.col1, this.col2, this.col3);
   }
   Mat33.prototype.SetM = function (m) {
      this.col1.SetV(m.col1);
      this.col2.SetV(m.col2);
      this.col3.SetV(m.col3);
   }
   Mat33.prototype.AddM = function (m) {
      this.col1.x += m.col1.x;
      this.col1.y += m.col1.y;
      this.col1.z += m.col1.z;
      this.col2.x += m.col2.x;
      this.col2.y += m.col2.y;
      this.col2.z += m.col2.z;
      this.col3.x += m.col3.x;
      this.col3.y += m.col3.y;
      this.col3.z += m.col3.z;
   }
   Mat33.prototype.SetIdentity = function () {
      this.col1.x = 1.0;
      this.col2.x = 0.0;
      this.col3.x = 0.0;
      this.col1.y = 0.0;
      this.col2.y = 1.0;
      this.col3.y = 0.0;
      this.col1.z = 0.0;
      this.col2.z = 0.0;
      this.col3.z = 1.0;
   }
   Mat33.prototype.SetZero = function () {
      this.col1.x = 0.0;
      this.col2.x = 0.0;
      this.col3.x = 0.0;
      this.col1.y = 0.0;
      this.col2.y = 0.0;
      this.col3.y = 0.0;
      this.col1.z = 0.0;
      this.col2.z = 0.0;
      this.col3.z = 0.0;
   }
   Mat33.prototype.Solve22 = function (out, bX, bY) {
      if (bX === undefined) bX = 0;
      if (bY === undefined) bY = 0;
      var a11 = this.col1.x;
      var a12 = this.col2.x;
      var a21 = this.col1.y;
      var a22 = this.col2.y;
      var det = a11 * a22 - a12 * a21;
      if (det != 0.0) {
         det = 1.0 / det;
      }
      out.x = det * (a22 * bX - a12 * bY);
      out.y = det * (a11 * bY - a21 * bX);
      return out;
   }
   Mat33.prototype.Solve33 = function (out, bX, bY, bZ) {
      if (bX === undefined) bX = 0;
      if (bY === undefined) bY = 0;
      if (bZ === undefined) bZ = 0;
      var a11 = this.col1.x;
      var a21 = this.col1.y;
      var a31 = this.col1.z;
      var a12 = this.col2.x;
      var a22 = this.col2.y;
      var a32 = this.col2.z;
      var a13 = this.col3.x;
      var a23 = this.col3.y;
      var a33 = this.col3.z;
      var det = a11 * (a22 * a33 - a32 * a23) + a21 * (a32 * a13 - a12 * a33) + a31 * (a12 * a23 - a22 * a13);
      if (det != 0.0) {
         det = 1.0 / det;
      }
      out.x = det * (bX * (a22 * a33 - a32 * a23) + bY * (a32 * a13 - a12 * a33) + bZ * (a12 * a23 - a22 * a13));
      out.y = det * (a11 * (bY * a33 - bZ * a23) + a21 * (bZ * a13 - bX * a33) + a31 * (bX * a23 - bY * a13));
      out.z = det * (a11 * (a22 * bZ - a32 * bY) + a21 * (a32 * bX - a12 * bZ) + a31 * (a12 * bY - a22 * bX));
      return out;
   }
   Math.Math = function () {};
   Math.IsValid = function (x) {
      if (x === undefined) x = 0;
      return isFinite(x);
   }
   Math.Dot = function (a, b) {
      return a.x * b.x + a.y * b.y;
   }
   Math.CrossVV = function (a, b) {
      return a.x * b.y - a.y * b.x;
   }
   Math.CrossVF = function (a, s) {
      if (s === undefined) s = 0;
      var v = new Vec2(s * a.y, (-s * a.x));
      return v;
   }
   Math.CrossFV = function (s, a) {
      if (s === undefined) s = 0;
      var v = new Vec2((-s * a.y), s * a.x);
      return v;
   }
   Math.MulMV = function (A, v) {
      var u = new Vec2(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y);
      return u;
   }
   Math.MulTMV = function (A, v) {
      var u = new Vec2(Math.Dot(v, A.col1), Math.Dot(v, A.col2));
      return u;
   }
   Math.MulX = function (T, v) {
      var a = Math.MulMV(T.R, v);
      a.x += T.position.x;
      a.y += T.position.y;
      return a;
   }
   Math.MulXT = function (T, v) {
      var a = Math.SubtractVV(v, T.position);
      var tX = (a.x * T.R.col1.x + a.y * T.R.col1.y);
      a.y = (a.x * T.R.col2.x + a.y * T.R.col2.y);
      a.x = tX;
      return a;
   }
   Math.AddVV = function (a, b) {
      var v = new Vec2(a.x + b.x, a.y + b.y);
      return v;
   }
   Math.SubtractVV = function (a, b) {
      var v = new Vec2(a.x - b.x, a.y - b.y);
      return v;
   }
   Math.Distance = function (a, b) {
      var cX = a.x - b.x;
      var cY = a.y - b.y;
      return Math.sqrt(cX * cX + cY * cY);
   }
   Math.DistanceSquared = function (a, b) {
      var cX = a.x - b.x;
      var cY = a.y - b.y;
      return (cX * cX + cY * cY);
   }
   Math.MulFV = function (s, a) {
      if (s === undefined) s = 0;
      var v = new Vec2(s * a.x, s * a.y);
      return v;
   }
   Math.AddMM = function (A, B) {
      var C = Mat22.FromVV(Math.AddVV(A.col1, B.col1), Math.AddVV(A.col2, B.col2));
      return C;
   }
   Math.MulMM = function (A, B) {
      var C = Mat22.FromVV(Math.MulMV(A, B.col1), Math.MulMV(A, B.col2));
      return C;
   }
   Math.MulTMM = function (A, B) {
      var c1 = new Vec2(Math.Dot(A.col1, B.col1), Math.Dot(A.col2, B.col1));
      var c2 = new Vec2(Math.Dot(A.col1, B.col2), Math.Dot(A.col2, B.col2));
      var C = Mat22.FromVV(c1, c2);
      return C;
   }
   Math.Abs = function (a) {
      if (a === undefined) a = 0;
      return a > 0.0 ? a : (-a);
   }
   Math.AbsV = function (a) {
      var b = new Vec2(Math.Abs(a.x), Math.Abs(a.y));
      return b;
   }
   Math.AbsM = function (A) {
      var B = Mat22.FromVV(Math.AbsV(A.col1), Math.AbsV(A.col2));
      return B;
   }
   Math.Min = function (a, b) {
      if (a === undefined) a = 0;
      if (b === undefined) b = 0;
      return a < b ? a : b;
   }
   Math.MinV = function (a, b) {
      var c = new Vec2(Math.Min(a.x, b.x), Math.Min(a.y, b.y));
      return c;
   }
   Math.Max = function (a, b) {
      if (a === undefined) a = 0;
      if (b === undefined) b = 0;
      return a > b ? a : b;
   }
   Math.MaxV = function (a, b) {
      var c = new Vec2(Math.Max(a.x, b.x), Math.Max(a.y, b.y));
      return c;
   }
   Math.Clamp = function (a, low, high) {
      if (a === undefined) a = 0;
      if (low === undefined) low = 0;
      if (high === undefined) high = 0;
      return a < low ? low : a > high ? high : a;
   }
   Math.ClampV = function (a, low, high) {
      return Math.MaxV(low, Math.MinV(a, high));
   }
   Math.Swap = function (a, b) {
      var tmp = a[0];
      a[0] = b[0];
      b[0] = tmp;
   }
   Math.Random = function () {
      return Math.random() * 2 - 1;
   }
   Math.RandomRange = function (lo, hi) {
      if (lo === undefined) lo = 0;
      if (hi === undefined) hi = 0;
      var r = Math.random();
      r = (hi - lo) * r + lo;
      return r;
   }
   Math.NextPowerOfTwo = function (x) {
      if (x === undefined) x = 0;
      x |= (x >> 1) & 0x7FFFFFFF;
      x |= (x >> 2) & 0x3FFFFFFF;
      x |= (x >> 4) & 0x0FFFFFFF;
      x |= (x >> 8) & 0x00FFFFFF;
      x |= (x >> 16) & 0x0000FFFF;
      return x + 1;
   }
   Math.IsPowerOfTwo = function (x) {
      if (x === undefined) x = 0;
      var result = x > 0 && (x & (x - 1)) == 0;
      return result;
   }
   Box2D.postDefs.push(function () {
      Box2D.Common.Math.Math.Vec2_zero = new Vec2(0.0, 0.0);
      Box2D.Common.Math.Math.Mat22_identity = Mat22.FromVV(new Vec2(1.0, 0.0), new Vec2(0.0, 1.0));
      Box2D.Common.Math.Math.Transforidentity = new Transform(Math.Vec2_zero, Math.Mat22_identity);
   });
   Sweep.Sweep = function () {
      this.localCenter = new Vec2();
      this.c0 = new Vec2;
      this.c = new Vec2();
   };
   Sweep.prototype.Set = function (other) {
      this.localCenter.SetV(other.localCenter);
      this.c0.SetV(other.c0);
      this.c.SetV(other.c);
      this.a0 = other.a0;
      this.a = other.a;
      this.t0 = other.t0;
   }
   Sweep.prototype.Copy = function () {
      var copy = new Sweep();
      copy.localCenter.SetV(this.localCenter);
      copy.c0.SetV(this.c0);
      copy.c.SetV(this.c);
      copy.a0 = this.a0;
      copy.a = this.a;
      copy.t0 = this.t0;
      return copy;
   }
   Sweep.prototype.GetTransform = function (xf, alpha) {
      if (alpha === undefined) alpha = 0;
      xf.position.x = (1.0 - alpha) * this.c0.x + alpha * this.c.x;
      xf.position.y = (1.0 - alpha) * this.c0.y + alpha * this.c.y;
      var angle = (1.0 - alpha) * this.a0 + alpha * this.a;
      xf.R.Set(angle);
      var tMat = xf.R;
      xf.position.x -= (tMat.col1.x * this.localCenter.x + tMat.col2.x * this.localCenter.y);
      xf.position.y -= (tMat.col1.y * this.localCenter.x + tMat.col2.y * this.localCenter.y);
   }
   Sweep.prototype.Advance = function (t) {
      if (t === undefined) t = 0;
      if (this.t0 < t && 1.0 - this.t0 > Number.MIN_VALUE) {
         var alpha = (t - this.t0) / (1.0 - this.t0);
         this.c0.x = (1.0 - alpha) * this.c0.x + alpha * this.c.x;
         this.c0.y = (1.0 - alpha) * this.c0.y + alpha * this.c.y;
         this.a0 = (1.0 - alpha) * this.a0 + alpha * this.a;
         this.t0 = t;
      }
   }
   Transform.Transform = function () {
      this.position = new Vec2;
      this.R = new Mat22();
   };
   Transform.prototype.Transform = function (pos, r) {
      if (pos === undefined) pos = null;
      if (r === undefined) r = null;
      if (pos) {
         this.position.SetV(pos);
         this.R.SetM(r);
      }
   }
   Transform.prototype.Initialize = function (pos, r) {
      this.position.SetV(pos);
      this.R.SetM(r);
   }
   Transform.prototype.SetIdentity = function () {
      this.position.SetZero();
      this.R.SetIdentity();
   }
   Transform.prototype.Set = function (x) {
      this.position.SetV(x.position);
      this.R.SetM(x.R);
   }
   Transform.prototype.GetAngle = function () {
      return Math.atan2(this.R.col1.y, this.R.col1.x);
   }
   Vec2.Vec2 = function () {};
   Vec2.prototype.Vec2 = function (x_, y_) {
      if (x_ === undefined) x_ = 0;
      if (y_ === undefined) y_ = 0;
      this.x = x_;
      this.y = y_;
   }
   Vec2.prototype.SetZero = function () {
      this.x = 0.0;
      this.y = 0.0;
   }
   Vec2.prototype.Set = function (x_, y_) {
      if (x_ === undefined) x_ = 0;
      if (y_ === undefined) y_ = 0;
      this.x = x_;
      this.y = y_;
   }
   Vec2.prototype.SetV = function (v) {
      this.x = v.x;
      this.y = v.y;
   }
   Vec2.prototype.GetNegative = function () {
      return new Vec2((-this.x), (-this.y));
   }
   Vec2.prototype.NegativeSelf = function () {
      this.x = (-this.x);
      this.y = (-this.y);
   }
   Vec2.Make = function (x_, y_) {
      if (x_ === undefined) x_ = 0;
      if (y_ === undefined) y_ = 0;
      return new Vec2(x_, y_);
   }
   Vec2.prototype.Copy = function () {
      return new Vec2(this.x, this.y);
   }
   Vec2.prototype.Add = function (v) {
      this.x += v.x;
      this.y += v.y;
   }
   Vec2.prototype.Subtract = function (v) {
      this.x -= v.x;
      this.y -= v.y;
   }
   Vec2.prototype.Multiply = function (a) {
      if (a === undefined) a = 0;
      this.x *= a;
      this.y *= a;
   }
   Vec2.prototype.MulM = function (A) {
      var tX = this.x;
      this.x = A.col1.x * tX + A.col2.x * this.y;
      this.y = A.col1.y * tX + A.col2.y * this.y;
   }
   Vec2.prototype.MulTM = function (A) {
      var tX = Math.Dot(this, A.col1);
      this.y = Math.Dot(this, A.col2);
      this.x = tX;
   }
   Vec2.prototype.CrossVF = function (s) {
      if (s === undefined) s = 0;
      var tX = this.x;
      this.x = s * this.y;
      this.y = (-s * tX);
   }
   Vec2.prototype.CrossFV = function (s) {
      if (s === undefined) s = 0;
      var tX = this.x;
      this.x = (-s * this.y);
      this.y = s * tX;
   }
   Vec2.prototype.MinV = function (b) {
      this.x = this.x < b.x ? this.x : b.x;
      this.y = this.y < b.y ? this.y : b.y;
   }
   Vec2.prototype.MaxV = function (b) {
      this.x = this.x > b.x ? this.x : b.x;
      this.y = this.y > b.y ? this.y : b.y;
   }
   Vec2.prototype.Abs = function () {
      if (this.x < 0) this.x = (-this.x);
      if (this.y < 0) this.y = (-this.y);
   }
   Vec2.prototype.Length = function () {
      return Math.sqrt(this.x * this.x + this.y * this.y);
   }
   Vec2.prototype.LengthSquared = function () {
      return (this.x * this.x + this.y * this.y);
   }
   Vec2.prototype.Normalize = function () {
      var length = Math.sqrt(this.x * this.x + this.y * this.y);
      if (length < Number.MIN_VALUE) {
         return 0.0;
      }
      var invLength = 1.0 / length;
      this.x *= invLength;
      this.y *= invLength;
      return length;
   }
   Vec2.prototype.IsValid = function () {
      return Math.IsValid(this.x) && Math.IsValid(this.y);
   }
   Vec3.Vec3 = function () {};
   Vec3.prototype.Vec3 = function (x, y, z) {
      if (x === undefined) x = 0;
      if (y === undefined) y = 0;
      if (z === undefined) z = 0;
      this.x = x;
      this.y = y;
      this.z = z;
   }
   Vec3.prototype.SetZero = function () {
      this.x = this.y = this.z = 0.0;
   }
   Vec3.prototype.Set = function (x, y, z) {
      if (x === undefined) x = 0;
      if (y === undefined) y = 0;
      if (z === undefined) z = 0;
      this.x = x;
      this.y = y;
      this.z = z;
   }
   Vec3.prototype.SetV = function (v) {
      this.x = v.x;
      this.y = v.y;
      this.z = v.z;
   }
   Vec3.prototype.GetNegative = function () {
      return new Vec3((-this.x), (-this.y), (-this.z));
   }
   Vec3.prototype.NegativeSelf = function () {
      this.x = (-this.x);
      this.y = (-this.y);
      this.z = (-this.z);
   }
   Vec3.prototype.Copy = function () {
      return new Vec3(this.x, this.y, this.z);
   }
   Vec3.prototype.Add = function (v) {
      this.x += v.x;
      this.y += v.y;
      this.z += v.z;
   }
   Vec3.prototype.Subtract = function (v) {
      this.x -= v.x;
      this.y -= v.y;
      this.z -= v.z;
   }
   Vec3.prototype.Multiply = function (a) {
      if (a === undefined) a = 0;
      this.x *= a;
      this.y *= a;
      this.z *= a;
   }
})();
(function () {
   var ControllerEdge = Box2D.Dynamics.Controllers.ControllerEdge,
      Mat22 = Box2D.Common.Math.Mat22,
      Mat33 = Box2D.Common.Math.Mat33,
      Math = Box2D.Common.Math.Math,
      Sweep = Box2D.Common.Math.Sweep,
      Transform = Box2D.Common.Math.Transform,
      Vec2 = Box2D.Common.Math.Vec2,
      Vec3 = Box2D.Common.Math.Vec3,
      Color = Box2D.Common.Color,
      internal = Box2D.Common.internal,
      Settings = Box2D.Common.Settings,
      AABB = Box2D.Collision.AABB,
      Bound = Box2D.Collision.Bound,
      BoundValues = Box2D.Collision.BoundValues,
      Collision = Box2D.Collision.Collision,
      ContactID = Box2D.Collision.ContactID,
      ContactPoint = Box2D.Collision.ContactPoint,
      Distance = Box2D.Collision.Distance,
      DistanceInput = Box2D.Collision.DistanceInput,
      DistanceOutput = Box2D.Collision.DistanceOutput,
      DistanceProxy = Box2D.Collision.DistanceProxy,
      DynamicTree = Box2D.Collision.DynamicTree,
      DynamicTreeBroadPhase = Box2D.Collision.DynamicTreeBroadPhase,
      DynamicTreeNode = Box2D.Collision.DynamicTreeNode,
      DynamicTreePair = Box2D.Collision.DynamicTreePair,
      Manifold = Box2D.Collision.Manifold,
      ManifoldPoint = Box2D.Collision.ManifoldPoint,
      Point = Box2D.Collision.Point,
      RayCastInput = Box2D.Collision.RayCastInput,
      RayCastOutput = Box2D.Collision.RayCastOutput,
      Segment = Box2D.Collision.Segment,
      SeparationFunction = Box2D.Collision.SeparationFunction,
      Simplex = Box2D.Collision.Simplex,
      SimplexCache = Box2D.Collision.SimplexCache,
      SimplexVertex = Box2D.Collision.SimplexVertex,
      TimeOfImpact = Box2D.Collision.TimeOfImpact,
      TOIInput = Box2D.Collision.TOIInput,
      WorldManifold = Box2D.Collision.WorldManifold,
      ClipVertex = Box2D.Collision.ClipVertex,
      Features = Box2D.Collision.Features,
      IBroadPhase = Box2D.Collision.IBroadPhase,
      CircleShape = Box2D.Collision.Shapes.CircleShape,
      EdgeChainDef = Box2D.Collision.Shapes.EdgeChainDef,
      EdgeShape = Box2D.Collision.Shapes.EdgeShape,
      MassData = Box2D.Collision.Shapes.MassData,
      PolygonShape = Box2D.Collision.Shapes.PolygonShape,
      Shape = Box2D.Collision.Shapes.Shape,
      Body = Box2D.Dynamics.Body,
      BodyDef = Box2D.Dynamics.BodyDef,
      ContactFilter = Box2D.Dynamics.ContactFilter,
      ContactImpulse = Box2D.Dynamics.ContactImpulse,
      ContactListener = Box2D.Dynamics.ContactListener,
      ContactManager = Box2D.Dynamics.ContactManager,
      DebugDraw = Box2D.Dynamics.DebugDraw,
      DestructionListener = Box2D.Dynamics.DestructionListener,
      FilterData = Box2D.Dynamics.FilterData,
      Fixture = Box2D.Dynamics.Fixture,
      FixtureDef = Box2D.Dynamics.FixtureDef,
      Island = Box2D.Dynamics.Island,
      TimeStep = Box2D.Dynamics.TimeStep,
      World = Box2D.Dynamics.World,
      CircleContact = Box2D.Dynamics.Contacts.CircleContact,
      Contact = Box2D.Dynamics.Contacts.Contact,
      ContactConstraint = Box2D.Dynamics.Contacts.ContactConstraint,
      ContactConstraintPoint = Box2D.Dynamics.Contacts.ContactConstraintPoint,
      ContactEdge = Box2D.Dynamics.Contacts.ContactEdge,
      ContactFactory = Box2D.Dynamics.Contacts.ContactFactory,
      ContactRegister = Box2D.Dynamics.Contacts.ContactRegister,
      ContactResult = Box2D.Dynamics.Contacts.ContactResult,
      ContactSolver = Box2D.Dynamics.Contacts.ContactSolver,
      EdgeAndCircleContact = Box2D.Dynamics.Contacts.EdgeAndCircleContact,
      NullContact = Box2D.Dynamics.Contacts.NullContact,
      PolyAndCircleContact = Box2D.Dynamics.Contacts.PolyAndCircleContact,
      PolyAndEdgeContact = Box2D.Dynamics.Contacts.PolyAndEdgeContact,
      PolygonContact = Box2D.Dynamics.Contacts.PolygonContact,
      PositionSolverManifold = Box2D.Dynamics.Contacts.PositionSolverManifold,
      Controller = Box2D.Dynamics.Controllers.Controller,
      DistanceJoint = Box2D.Dynamics.Joints.DistanceJoint,
      DistanceJointDef = Box2D.Dynamics.Joints.DistanceJointDef,
      FrictionJoint = Box2D.Dynamics.Joints.FrictionJoint,
      FrictionJointDef = Box2D.Dynamics.Joints.FrictionJointDef,
      GearJoint = Box2D.Dynamics.Joints.GearJoint,
      GearJointDef = Box2D.Dynamics.Joints.GearJointDef,
      Jacobian = Box2D.Dynamics.Joints.Jacobian,
      Joint = Box2D.Dynamics.Joints.Joint,
      JointDef = Box2D.Dynamics.Joints.JointDef,
      JointEdge = Box2D.Dynamics.Joints.JointEdge,
      LineJoint = Box2D.Dynamics.Joints.LineJoint,
      LineJointDef = Box2D.Dynamics.Joints.LineJointDef,
      MouseJoint = Box2D.Dynamics.Joints.MouseJoint,
      MouseJointDef = Box2D.Dynamics.Joints.MouseJointDef,
      PrismaticJoint = Box2D.Dynamics.Joints.PrismaticJoint,
      PrismaticJointDef = Box2D.Dynamics.Joints.PrismaticJointDef,
      PulleyJoint = Box2D.Dynamics.Joints.PulleyJoint,
      PulleyJointDef = Box2D.Dynamics.Joints.PulleyJointDef,
      RevoluteJoint = Box2D.Dynamics.Joints.RevoluteJoint,
      RevoluteJointDef = Box2D.Dynamics.Joints.RevoluteJointDef,
      WeldJoint = Box2D.Dynamics.Joints.WeldJoint,
      WeldJointDef = Box2D.Dynamics.Joints.WeldJointDef;

   Body.Body = function () {
      this.xf = new Transform();
      this.sweep = new Sweep();
      this.linearVelocity = new Vec2();
      this.force = new Vec2();
   };
   Body.prototype.connectEdges = function (s1, s2, angle1) {
      if (angle1 === undefined) angle1 = 0;
      var angle2 = Math.atan2(s2.GetDirectionVector().y, s2.GetDirectionVector().x);
      var coreOffset = Math.tan((angle2 - angle1) * 0.5);
      var core = Math.MulFV(coreOffset, s2.GetDirectionVector());
      core = Math.SubtractVV(core, s2.GetNormalVector());
      core = Math.MulFV(Settings._toiSlop, core);
      core = Math.AddVV(core, s2.GetVertex1());
      var cornerDir = Math.AddVV(s1.GetDirectionVector(), s2.GetDirectionVector());
      cornerDir.Normalize();
      var convex = Math.Dot(s1.GetDirectionVector(), s2.GetNormalVector()) > 0.0;
      s1.SetNextEdge(s2, core, cornerDir, convex);
      s2.SetPrevEdge(s1, core, cornerDir, convex);
      return angle2;
   }
   Body.prototype.CreateFixture = function (def) {
      if (this.world.IsLocked() == true) {
         return null;
      }
      var fixture = new Fixture();
      fixture.Create(this, this.xf, def);
      if (this.flags & Body.e_activeFlag) {
         var broadPhase = this.world.contactManager.broadPhase;
         fixture.CreateProxy(broadPhase, this.xf);
      }
      fixture.next = this.fixtureList;
      this.fixtureList = fixture;
      ++this.fixtureCount;
      fixture.body = this;
      if (fixture.density > 0.0) {
         this.ResetMassData();
      }
      this.world.flags |= World.e_newFixture;
      return fixture;
   }
   Body.prototype.CreateFixture2 = function (shape, density) {
      if (density === undefined) density = 0.0;
      var def = new FixtureDef();
      def.shape = shape;
      def.density = density;
      return this.CreateFixture(def);
   }
   Body.prototype.DestroyFixture = function (fixture) {
      if (this.world.IsLocked() == true) {
         return;
      }
      var node = this.fixtureList;
      var ppF = null;
      var found = false;
      while (node != null) {
         if (node == fixture) {
            if (ppF) ppF.next = fixture.next;
            else this.fixtureList = fixture.next;
            found = true;
            break;
         }
         ppF = node;
         node = node.next;
      }
      var edge = this.contactList;
      while (edge) {
         var c = edge.contact;
         edge = edge.next;
         var fixtureA = c.GetFixtureA();
         var fixtureB = c.GetFixtureB();
         if (fixture == fixtureA || fixture == fixtureB) {
            this.world.contactManager.Destroy(c);
         }
      }
      if (this.flags & Body.e_activeFlag) {
         var broadPhase = this.world.contactManager.broadPhase;
         fixture.DestroyProxy(broadPhase);
      }
      else {}
      fixture.Destroy();
      fixture.body = null;
      fixture.next = null;
      --this.fixtureCount;
      this.ResetMassData();
   }
   Body.prototype.SetPositionAndAngle = function (position, angle) {
      if (angle === undefined) angle = 0;
      var f;
      if (this.world.IsLocked() == true) {
         return;
      }
      this.xf.R.Set(angle);
      this.xf.position.SetV(position);
      var tMat = this.xf.R;
      var tVec = this.sweep.localCenter;
      this.sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      this.sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      this.sweep.c.x += this.xf.position.x;
      this.sweep.c.y += this.xf.position.y;
      this.sweep.c0.SetV(this.sweep.c);
      this.sweep.a0 = this.sweep.a = angle;
      var broadPhase = this.world.contactManager.broadPhase;
      for (f = this.fixtureList;
      f; f = f.next) {
         f.Synchronize(broadPhase, this.xf, this.xf);
      }
      this.world.contactManager.FindNewContacts();
   }
   Body.prototype.SetTransform = function (xf) {
      this.SetPositionAndAngle(xf.position, xf.GetAngle());
   }
   Body.prototype.GetTransform = function () {
      return this.xf;
   }
   Body.prototype.GetPosition = function () {
      return this.xf.position;
   }
   Body.prototype.SetPosition = function (position) {
      this.SetPositionAndAngle(position, this.GetAngle());
   }
   Body.prototype.GetAngle = function () {
      return this.sweep.a;
   }
   Body.prototype.SetAngle = function (angle) {
      if (angle === undefined) angle = 0;
      this.SetPositionAndAngle(this.GetPosition(), angle);
   }
   Body.prototype.GetWorldCenter = function () {
      return this.sweep.c;
   }
   Body.prototype.GetLocalCenter = function () {
      return this.sweep.localCenter;
   }
   Body.prototype.SetLinearVelocity = function (v) {
      if (this.type == Body._staticBody) {
         return;
      }
      this.linearVelocity.SetV(v);
   }
   Body.prototype.GetLinearVelocity = function () {
      return this.linearVelocity;
   }
   Body.prototype.SetAngularVelocity = function (omega) {
      if (omega === undefined) omega = 0;
      if (this.type == Body._staticBody) {
         return;
      }
      this.angularVelocity = omega;
   }
   Body.prototype.GetAngularVelocity = function () {
      return this.angularVelocity;
   }
   Body.prototype.GetDefinition = function () {
      var bd = new BodyDef();
      bd.type = this.GetType();
      bd.allowSleep = (this.flags & Body.e_allowSleepFlag) == Body.e_allowSleepFlag;
      bd.angle = this.GetAngle();
      bd.angularDamping = this.angularDamping;
      bd.angularVelocity = this.angularVelocity;
      bd.fixedRotation = (this.flags & Body.e_fixedRotationFlag) == Body.e_fixedRotationFlag;
      bd.bullet = (this.flags & Body.e_bulletFlag) == Body.e_bulletFlag;
      bd.awake = (this.flags & Body.e_awakeFlag) == Body.e_awakeFlag;
      bd.linearDamping = this.linearDamping;
      bd.linearVelocity.SetV(this.GetLinearVelocity());
      bd.position = this.GetPosition();
      bd.userData = this.GetUserData();
      return bd;
   }
   Body.prototype.ApplyForce = function (force, point) {
      if (this.type != Body._dynamicBody) {
         return;
      }
      if (this.IsAwake() == false) {
         this.SetAwake(true);
      }
      this.force.x += force.x;
      this.force.y += force.y;
      this.torque += ((point.x - this.sweep.c.x) * force.y - (point.y - this.sweep.c.y) * force.x);
   }
   Body.prototype.ApplyTorque = function (torque) {
      if (torque === undefined) torque = 0;
      if (this.type != Body._dynamicBody) {
         return;
      }
      if (this.IsAwake() == false) {
         this.SetAwake(true);
      }
      this.torque += torque;
   }
   Body.prototype.ApplyImpulse = function (impulse, point) {
      if (this.type != Body._dynamicBody) {
         return;
      }
      if (this.IsAwake() == false) {
         this.SetAwake(true);
      }
      this.linearVelocity.x += this.invMass * impulse.x;
      this.linearVelocity.y += this.invMass * impulse.y;
      this.angularVelocity += this.invI * ((point.x - this.sweep.c.x) * impulse.y - (point.y - this.sweep.c.y) * impulse.x);
   }
   Body.prototype.Split = function (callback) {
      var linearVelocity = this.GetLinearVelocity().Copy();
      var angularVelocity = this.GetAngularVelocity();
      var center = this.GetWorldCenter();
      var body1 = this;
      var body2 = this.world.CreateBody(this.GetDefinition());
      var prev;
      for (var f = body1.fixtureList; f;) {
         if (callback(f)) {
            var next = f.next;
            if (prev) {
               prev.next = next;
            }
            else {
               body1.fixtureList = next;
            }
            body1.fixtureCount--;
            f.next = body2.fixtureList;
            body2.fixtureList = f;
            body2.fixtureCount++;
            f.body = body2;
            f = next;
         }
         else {
            prev = f;
            f = f.next;
         }
      }
      body1.ResetMassData();
      body2.ResetMassData();
      var center1 = body1.GetWorldCenter();
      var center2 = body2.GetWorldCenter();
      var velocity1 = Math.AddVV(linearVelocity, Math.CrossFV(angularVelocity, Math.SubtractVV(center1, center)));
      var velocity2 = Math.AddVV(linearVelocity, Math.CrossFV(angularVelocity, Math.SubtractVV(center2, center)));
      body1.SetLinearVelocity(velocity1);
      body2.SetLinearVelocity(velocity2);
      body1.SetAngularVelocity(angularVelocity);
      body2.SetAngularVelocity(angularVelocity);
      body1.SynchronizeFixtures();
      body2.SynchronizeFixtures();
      return body2;
   }
   Body.prototype.Merge = function (other) {
      var f;
      for (f = other.fixtureList;
      f;) {
         var next = f.next;
         other.fixtureCount--;
         f.next = this.fixtureList;
         this.fixtureList = f;
         this.fixtureCount++;
         f.body = body2;
         f = next;
      }
      body1.fixtureCount = 0;
      var body1 = this;
      var body2 = other;
      var center1 = body1.GetWorldCenter();
      var center2 = body2.GetWorldCenter();
      var velocity1 = body1.GetLinearVelocity().Copy();
      var velocity2 = body2.GetLinearVelocity().Copy();
      var angular1 = body1.GetAngularVelocity();
      var angular = body2.GetAngularVelocity();
      body1.ResetMassData();
      this.SynchronizeFixtures();
   }
   Body.prototype.GetMass = function () {
      return this.mass;
   }
   Body.prototype.GetInertia = function () {
      return this.I;
   }
   Body.prototype.GetMassData = function (data) {
      data.mass = this.mass;
      data.I = this.I;
      data.center.SetV(this.sweep.localCenter);
   }
   Body.prototype.SetMassData = function (massData) {
      Settings.Assert(this.world.IsLocked() == false);
      if (this.world.IsLocked() == true) {
         return;
      }
      if (this.type != Body._dynamicBody) {
         return;
      }
      this.invMass = 0.0;
      this.I = 0.0;
      this.invI = 0.0;
      this.mass = massData.mass;
      if (this.mass <= 0.0) {
         this.mass = 1.0;
      }
      this.invMass = 1.0 / this.mass;
      if (massData.I > 0.0 && (this.flags & Body.e_fixedRotationFlag) == 0) {
         this.I = massData.I - this.mass * (massData.center.x * massData.center.x + massData.center.y * massData.center.y);
         this.invI = 1.0 / this.I;
      }
      var oldCenter = this.sweep.c.Copy();
      this.sweep.localCenter.SetV(massData.center);
      this.sweep.c0.SetV(Math.MulX(this.xf, this.sweep.localCenter));
      this.sweep.c.SetV(this.sweep.c0);
      this.linearVelocity.x += this.angularVelocity * (-(this.sweep.c.y - oldCenter.y));
      this.linearVelocity.y += this.angularVelocity * (+(this.sweep.c.x - oldCenter.x));
   }
   Body.prototype.ResetMassData = function () {
      this.mass = 0.0;
      this.invMass = 0.0;
      this.I = 0.0;
      this.invI = 0.0;
      this.sweep.localCenter.SetZero();
      if (this.type == Body._staticBody || this.type == Body._kinematicBody) {
         return;
      }
      var center = Vec2.Make(0, 0);
      for (var f = this.fixtureList; f; f = f.next) {
         if (f.density == 0.0) {
            continue;
         }
         var massData = f.GetMassData();
         this.mass += massData.mass;
         center.x += massData.center.x * massData.mass;
         center.y += massData.center.y * massData.mass;
         this.I += massData.I;
      }
      if (this.mass > 0.0) {
         this.invMass = 1.0 / this.mass;
         center.x *= this.invMass;
         center.y *= this.invMass;
      }
      else {
         this.mass = 1.0;
         this.invMass = 1.0;
      }
      if (this.I > 0.0 && (this.flags & Body.e_fixedRotationFlag) == 0) {
         this.I -= this.mass * (center.x * center.x + center.y * center.y);
         this.I *= this.inertiaScale;
         Settings.Assert(this.I > 0);
         this.invI = 1.0 / this.I;
      }
      else {
         this.I = 0.0;
         this.invI = 0.0;
      }
      var oldCenter = this.sweep.c.Copy();
      this.sweep.localCenter.SetV(center);
      this.sweep.c0.SetV(Math.MulX(this.xf, this.sweep.localCenter));
      this.sweep.c.SetV(this.sweep.c0);
      this.linearVelocity.x += this.angularVelocity * (-(this.sweep.c.y - oldCenter.y));
      this.linearVelocity.y += this.angularVelocity * (+(this.sweep.c.x - oldCenter.x));
   }
   Body.prototype.GetWorldPoint = function (localPoint) {
      var A = this.xf.R;
      var u = new Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y, A.col1.y * localPoint.x + A.col2.y * localPoint.y);
      u.x += this.xf.position.x;
      u.y += this.xf.position.y;
      return u;
   }
   Body.prototype.GetWorldVector = function (localVector) {
      return Math.MulMV(this.xf.R, localVector);
   }
   Body.prototype.GetLocalPoint = function (worldPoint) {
      return Math.MulXT(this.xf, worldPoint);
   }
   Body.prototype.GetLocalVector = function (worldVector) {
      return Math.MulTMV(this.xf.R, worldVector);
   }
   Body.prototype.GetLinearVelocityFromWorldPoint = function (worldPoint) {
      return new Vec2(this.linearVelocity.x - this.angularVelocity * (worldPoint.y - this.sweep.c.y), this.linearVelocity.y + this.angularVelocity * (worldPoint.x - this.sweep.c.x));
   }
   Body.prototype.GetLinearVelocityFromLocalPoint = function (localPoint) {
      var A = this.xf.R;
      var worldPoint = new Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y, A.col1.y * localPoint.x + A.col2.y * localPoint.y);
      worldPoint.x += this.xf.position.x;
      worldPoint.y += this.xf.position.y;
      return new Vec2(this.linearVelocity.x - this.angularVelocity * (worldPoint.y - this.sweep.c.y), this.linearVelocity.y + this.angularVelocity * (worldPoint.x - this.sweep.c.x));
   }
   Body.prototype.GetLinearDamping = function () {
      return this.linearDamping;
   }
   Body.prototype.SetLinearDamping = function (linearDamping) {
      if (linearDamping === undefined) linearDamping = 0;
      this.linearDamping = linearDamping;
   }
   Body.prototype.GetAngularDamping = function () {
      return this.angularDamping;
   }
   Body.prototype.SetAngularDamping = function (angularDamping) {
      if (angularDamping === undefined) angularDamping = 0;
      this.angularDamping = angularDamping;
   }
   Body.prototype.SetType = function (type) {
      if (type === undefined) type = 0;
      if (this.type == type) {
         return;
      }
      this.type = type;
      this.ResetMassData();
      if (this.type == Body._staticBody) {
         this.linearVelocity.SetZero();
         this.angularVelocity = 0.0;
      }
      this.SetAwake(true);
      this.force.SetZero();
      this.torque = 0.0;
      for (var ce = this.contactList; ce; ce = ce.next) {
         ce.contact.FlagForFiltering();
      }
   }
   Body.prototype.GetType = function () {
      return this.type;
   }
   Body.prototype.SetBullet = function (flag) {
      if (flag) {
         this.flags |= Body.e_bulletFlag;
      }
      else {
         this.flags &= ~Body.e_bulletFlag;
      }
   }
   Body.prototype.IsBullet = function () {
      return (this.flags & Body.e_bulletFlag) == Body.e_bulletFlag;
   }
   Body.prototype.SetSleepingAllowed = function (flag) {
      if (flag) {
         this.flags |= Body.e_allowSleepFlag;
      }
      else {
         this.flags &= ~Body.e_allowSleepFlag;
         this.SetAwake(true);
      }
   }
   Body.prototype.SetAwake = function (flag) {
      if (flag) {
         this.flags |= Body.e_awakeFlag;
         this.sleepTime = 0.0;
      }
      else {
         this.flags &= ~Body.e_awakeFlag;
         this.sleepTime = 0.0;
         this.linearVelocity.SetZero();
         this.angularVelocity = 0.0;
         this.force.SetZero();
         this.torque = 0.0;
      }
   }
   Body.prototype.IsAwake = function () {
      return (this.flags & Body.e_awakeFlag) == Body.e_awakeFlag;
   }
   Body.prototype.SetFixedRotation = function (fixed) {
      if (fixed) {
         this.flags |= Body.e_fixedRotationFlag;
      }
      else {
         this.flags &= ~Body.e_fixedRotationFlag;
      }
      this.ResetMassData();
   }
   Body.prototype.IsFixedRotation = function () {
      return (this.flags & Body.e_fixedRotationFlag) == Body.e_fixedRotationFlag;
   }
   Body.prototype.SetActive = function (flag) {
      if (flag == this.IsActive()) {
         return;
      }
      var broadPhase;
      var f;
      if (flag) {
         this.flags |= Body.e_activeFlag;
         broadPhase = this.world.contactManager.broadPhase;
         for (f = this.fixtureList;
         f; f = f.next) {
            f.CreateProxy(broadPhase, this.xf);
         }
      }
      else {
         this.flags &= ~Body.e_activeFlag;
         broadPhase = this.world.contactManager.broadPhase;
         for (f = this.fixtureList;
         f; f = f.next) {
            f.DestroyProxy(broadPhase);
         }
         var ce = this.contactList;
         while (ce) {
            var ce0 = ce;
            ce = ce.next;
            this.world.contactManager.Destroy(ce0.contact);
         }
         this.contactList = null;
      }
   }
   Body.prototype.IsActive = function () {
      return (this.flags & Body.e_activeFlag) == Body.e_activeFlag;
   }
   Body.prototype.IsSleepingAllowed = function () {
      return (this.flags & Body.e_allowSleepFlag) == Body.e_allowSleepFlag;
   }
   Body.prototype.GetFixtureList = function () {
      return this.fixtureList;
   }
   Body.prototype.GetJointList = function () {
      return this.jointList;
   }
   Body.prototype.GetControllerList = function () {
      return this.controllerList;
   }
   Body.prototype.GetContactList = function () {
      return this.contactList;
   }
   Body.prototype.GetNext = function () {
      return this.next;
   }
   Body.prototype.GetUserData = function () {
      return this.userData;
   }
   Body.prototype.SetUserData = function (data) {
      this.userData = data;
   }
   Body.prototype.GetWorld = function () {
      return this.world;
   }
   Body.prototype.Body = function (bd, world) {
      this.flags = 0;
      if (bd.bullet) {
         this.flags |= Body.e_bulletFlag;
      }
      if (bd.fixedRotation) {
         this.flags |= Body.e_fixedRotationFlag;
      }
      if (bd.allowSleep) {
         this.flags |= Body.e_allowSleepFlag;
      }
      if (bd.awake) {
         this.flags |= Body.e_awakeFlag;
      }
      if (bd.active) {
         this.flags |= Body.e_activeFlag;
      }
      this.world = world;
      this.xf.position.SetV(bd.position);
      this.xf.R.Set(bd.angle);
      this.sweep.localCenter.SetZero();
      this.sweep.t0 = 1.0;
      this.sweep.a0 = this.sweep.a = bd.angle;
      var tMat = this.xf.R;
      var tVec = this.sweep.localCenter;
      this.sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      this.sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      this.sweep.c.x += this.xf.position.x;
      this.sweep.c.y += this.xf.position.y;
      this.sweep.c0.SetV(this.sweep.c);
      this.jointList = null;
      this.controllerList = null;
      this.contactList = null;
      this.controllerCount = 0;
      this.prev = null;
      this.next = null;
      this.linearVelocity.SetV(bd.linearVelocity);
      this.angularVelocity = bd.angularVelocity;
      this.linearDamping = bd.linearDamping;
      this.angularDamping = bd.angularDamping;
      this.force.Set(0.0, 0.0);
      this.torque = 0.0;
      this.sleepTime = 0.0;
      this.type = bd.type;
      if (this.type == Body._dynamicBody) {
         this.mass = 1.0;
         this.invMass = 1.0;
      }
      else {
         this.mass = 0.0;
         this.invMass = 0.0;
      }
      this.I = 0.0;
      this.invI = 0.0;
      this.inertiaScale = bd.inertiaScale;
      this.userData = bd.userData;
      this.fixtureList = null;
      this.fixtureCount = 0;
   }
   Body.prototype.SynchronizeFixtures = function () {
      var xf1 = Body.s_xf1;
      xf1.R.Set(this.sweep.a0);
      var tMat = xf1.R;
      var tVec = this.sweep.localCenter;
      xf1.position.x = this.sweep.c0.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      xf1.position.y = this.sweep.c0.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      var f;
      var broadPhase = this.world.contactManager.broadPhase;
      for (f = this.fixtureList;
      f; f = f.next) {
         f.Synchronize(broadPhase, xf1, this.xf);
      }
   }
   Body.prototype.SynchronizeTransform = function () {
      this.xf.R.Set(this.sweep.a);
      var tMat = this.xf.R;
      var tVec = this.sweep.localCenter;
      this.xf.position.x = this.sweep.c.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      this.xf.position.y = this.sweep.c.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
   }
   Body.prototype.ShouldCollide = function (other) {
      if (this.type != Body._dynamicBody && other.type != Body._dynamicBody) {
         return false;
      }
      for (var jn = this.jointList; jn; jn = jn.next) {
         if (jn.other == other) if (jn.joint.collideConnected == false) {
            return false;
         }
      }
      return true;
   }
   Body.prototype.Advance = function (t) {
      if (t === undefined) t = 0;
      this.sweep.Advance(t);
      this.sweep.c.SetV(this.sweep.c0);
      this.sweep.a = this.sweep.a0;
      this.SynchronizeTransform();
   }
   Box2D.postDefs.push(function () {
      Box2D.Dynamics.Body.s_xf1 = new Transform();
      Box2D.Dynamics.Body.e_islandFlag = 0x0001;
      Box2D.Dynamics.Body.e_awakeFlag = 0x0002;
      Box2D.Dynamics.Body.e_allowSleepFlag = 0x0004;
      Box2D.Dynamics.Body.e_bulletFlag = 0x0008;
      Box2D.Dynamics.Body.e_fixedRotationFlag = 0x0010;
      Box2D.Dynamics.Body.e_activeFlag = 0x0020;
      Box2D.Dynamics.Body._staticBody = 0;
      Box2D.Dynamics.Body._kinematicBody = 1;
      Box2D.Dynamics.Body._dynamicBody = 2;
   });
   BodyDef.BodyDef = function () {
      this.position = new Vec2();
      this.linearVelocity = new Vec2();
   };
   BodyDef.prototype.BodyDef = function () {
      this.userData = null;
      this.position.Set(0.0, 0.0);
      this.angle = 0.0;
      this.linearVelocity.Set(0, 0);
      this.angularVelocity = 0.0;
      this.linearDamping = 0.0;
      this.angularDamping = 0.0;
      this.allowSleep = true;
      this.awake = true;
      this.fixedRotation = false;
      this.bullet = false;
      this.type = Body._staticBody;
      this.active = true;
      this.inertiaScale = 1.0;
   }
   ContactFilter.ContactFilter = function () {};
   ContactFilter.prototype.ShouldCollide = function (fixtureA, fixtureB) {
      var filter1 = fixtureA.GetFilterData();
      var filter2 = fixtureB.GetFilterData();
      if (filter1.groupIndex == filter2.groupIndex && filter1.groupIndex != 0) {
         return filter1.groupIndex > 0;
      }
      var collide = (filter1.maskBits & filter2.categoryBits) != 0 && (filter1.categoryBits & filter2.maskBits) != 0;
      return collide;
   }
   ContactFilter.prototype.RayCollide = function (userData, fixture) {
      if (!userData) return true;
      return this.ShouldCollide((userData instanceof Fixture ? userData : null), fixture);
   }
   Box2D.postDefs.push(function () {
      Box2D.Dynamics.ContactFilter._defaultFilter = new ContactFilter();
   });
   ContactImpulse.ContactImpulse = function () {
      this.normalImpulses = new Vector_a2j_Number(Settings._maxManifoldPoints);
      this.tangentImpulses = new Vector_a2j_Number(Settings._maxManifoldPoints);
   };
   ContactListener.ContactListener = function () {};
   ContactListener.prototype.BeginContact = function (contact) {}
   ContactListener.prototype.EndContact = function (contact) {}
   ContactListener.prototype.PreSolve = function (contact, oldManifold) {}
   ContactListener.prototype.PostSolve = function (contact, impulse) {}
   Box2D.postDefs.push(function () {
      Box2D.Dynamics.ContactListener._defaultListener = new ContactListener();
   });
   ContactManager.ContactManager = function () {};
   ContactManager.prototype.ContactManager = function () {
      this.world = null;
      this.contactCount = 0;
      this.contactFilter = ContactFilter._defaultFilter;
      this.contactListener = ContactListener._defaultListener;
      this.contactFactory = new ContactFactory(this.allocator);
      this.broadPhase = new DynamicTreeBroadPhase();
   }
   ContactManager.prototype.AddPair = function (proxyUserDataA, proxyUserDataB) {
      var fixtureA = (proxyUserDataA instanceof Fixture ? proxyUserDataA : null);
      var fixtureB = (proxyUserDataB instanceof Fixture ? proxyUserDataB : null);
      var bodyA = fixtureA.GetBody();
      var bodyB = fixtureB.GetBody();
      if (bodyA == bodyB) return;
      var edge = bodyB.GetContactList();
      while (edge) {
         if (edge.other == bodyA) {
            var fA = edge.contact.GetFixtureA();
            var fB = edge.contact.GetFixtureB();
            if (fA == fixtureA && fB == fixtureB) return;
            if (fA == fixtureB && fB == fixtureA) return;
         }
         edge = edge.next;
      }
      if (bodyB.ShouldCollide(bodyA) == false) {
         return;
      }
      if (this.contactFilter.ShouldCollide(fixtureA, fixtureB) == false) {
         return;
      }
      var c = this.contactFactory.Create(fixtureA, fixtureB);
      fixtureA = c.GetFixtureA();
      fixtureB = c.GetFixtureB();
      bodyA = fixtureA.body;
      bodyB = fixtureB.body;
      c.prev = null;
      c.next = this.world.contactList;
      if (this.world.contactList != null) {
         this.world.contactList.prev = c;
      }
      this.world.contactList = c;
      c.nodeA.contact = c;
      c.nodeA.other = bodyB;
      c.nodeA.prev = null;
      c.nodeA.next = bodyA.contactList;
      if (bodyA.contactList != null) {
         bodyA.contactList.prev = c.nodeA;
      }
      bodyA.contactList = c.nodeA;
      c.nodeB.contact = c;
      c.nodeB.other = bodyA;
      c.nodeB.prev = null;
      c.nodeB.next = bodyB.contactList;
      if (bodyB.contactList != null) {
         bodyB.contactList.prev = c.nodeB;
      }
      bodyB.contactList = c.nodeB;
      ++this.world.contactCount;
      return;
   }
   ContactManager.prototype.FindNewContacts = function () {
      this.broadPhase.UpdatePairs(Box2D.generateCallback(this, this.AddPair));
   }
   ContactManager.prototype.Destroy = function (c) {
      var fixtureA = c.GetFixtureA();
      var fixtureB = c.GetFixtureB();
      var bodyA = fixtureA.GetBody();
      var bodyB = fixtureB.GetBody();
      if (c.IsTouching()) {
         this.contactListener.EndContact(c);
      }
      if (c.prev) {
         c.prev.next = c.next;
      }
      if (c.next) {
         c.next.prev = c.prev;
      }
      if (c == this.world.contactList) {
         this.world.contactList = c.next;
      }
      if (c.nodeA.prev) {
         c.nodeA.prev.next = c.nodeA.next;
      }
      if (c.nodeA.next) {
         c.nodeA.next.prev = c.nodeA.prev;
      }
      if (c.nodeA == bodyA.contactList) {
         bodyA.contactList = c.nodeA.next;
      }
      if (c.nodeB.prev) {
         c.nodeB.prev.next = c.nodeB.next;
      }
      if (c.nodeB.next) {
         c.nodeB.next.prev = c.nodeB.prev;
      }
      if (c.nodeB == bodyB.contactList) {
         bodyB.contactList = c.nodeB.next;
      }
      this.contactFactory.Destroy(c);
      --this.contactCount;
   }
   ContactManager.prototype.Collide = function () {
      var c = this.world.contactList;
      while (c) {
         var fixtureA = c.GetFixtureA();
         var fixtureB = c.GetFixtureB();
         var bodyA = fixtureA.GetBody();
         var bodyB = fixtureB.GetBody();
         if (bodyA.IsAwake() == false && bodyB.IsAwake() == false) {
            c = c.GetNext();
            continue;
         }
         if (c.flags & Contact.e_filterFlag) {
            if (bodyB.ShouldCollide(bodyA) == false) {
               var cNuke = c;
               c = cNuke.GetNext();
               this.Destroy(cNuke);
               continue;
            }
            if (this.contactFilter.ShouldCollide(fixtureA, fixtureB) == false) {
               cNuke = c;
               c = cNuke.GetNext();
               this.Destroy(cNuke);
               continue;
            }
            c.flags &= ~Contact.e_filterFlag;
         }
         var proxyA = fixtureA.proxy;
         var proxyB = fixtureB.proxy;
         var overlap = this.broadPhase.TestOverlap(proxyA, proxyB);
         if (overlap == false) {
            cNuke = c;
            c = cNuke.GetNext();
            this.Destroy(cNuke);
            continue;
         }
         c.Update(this.contactListener);
         c = c.GetNext();
      }
   }
   Box2D.postDefs.push(function () {
      Box2D.Dynamics.ContactManager.s_evalCP = new ContactPoint();
   });
   DebugDraw.DebugDraw = function () {};
   DebugDraw.prototype.DebugDraw = function () {}
   DebugDraw.prototype.SetFlags = function (flags) {
      if (flags === undefined) flags = 0;
   }
   DebugDraw.prototype.GetFlags = function () {}
   DebugDraw.prototype.AppendFlags = function (flags) {
      if (flags === undefined) flags = 0;
   }
   DebugDraw.prototype.ClearFlags = function (flags) {
      if (flags === undefined) flags = 0;
   }
   DebugDraw.prototype.SetSprite = function (sprite) {}
   DebugDraw.prototype.GetSprite = function () {}
   DebugDraw.prototype.SetDrawScale = function (drawScale) {
      if (drawScale === undefined) drawScale = 0;
   }
   DebugDraw.prototype.GetDrawScale = function () {}
   DebugDraw.prototype.SetLineThickness = function (lineThickness) {
      if (lineThickness === undefined) lineThickness = 0;
   }
   DebugDraw.prototype.GetLineThickness = function () {}
   DebugDraw.prototype.SetAlpha = function (alpha) {
      if (alpha === undefined) alpha = 0;
   }
   DebugDraw.prototype.GetAlpha = function () {}
   DebugDraw.prototype.SetFillAlpha = function (alpha) {
      if (alpha === undefined) alpha = 0;
   }
   DebugDraw.prototype.GetFillAlpha = function () {}
   DebugDraw.prototype.SetXFormScale = function (xformScale) {
      if (xformScale === undefined) xformScale = 0;
   }
   DebugDraw.prototype.GetXFormScale = function () {}
   DebugDraw.prototype.DrawPolygon = function (vertices, vertexCount, color) {
      if (vertexCount === undefined) vertexCount = 0;
   }
   DebugDraw.prototype.DrawSolidPolygon = function (vertices, vertexCount, color) {
      if (vertexCount === undefined) vertexCount = 0;
   }
   DebugDraw.prototype.DrawCircle = function (center, radius, color) {
      if (radius === undefined) radius = 0;
   }
   DebugDraw.prototype.DrawSolidCircle = function (center, radius, axis, color) {
      if (radius === undefined) radius = 0;
   }
   DebugDraw.prototype.DrawSegment = function (p1, p2, color) {}
   DebugDraw.prototype.DrawTransform = function (xf) {}
   Box2D.postDefs.push(function () {
      Box2D.Dynamics.DebugDraw.e_shapeBit = 0x0001;
      Box2D.Dynamics.DebugDraw.e_jointBit = 0x0002;
      Box2D.Dynamics.DebugDraw.e_aabbBit = 0x0004;
      Box2D.Dynamics.DebugDraw.e_pairBit = 0x0008;
      Box2D.Dynamics.DebugDraw.e_centerOfMassBit = 0x0010;
      Box2D.Dynamics.DebugDraw.e_controllerBit = 0x0020;
   });
   DestructionListener.DestructionListener = function () {};
   DestructionListener.prototype.SayGoodbyeJoint = function (joint) {}
   DestructionListener.prototype.SayGoodbyeFixture = function (fixture) {}
   FilterData.FilterData = function () {
      this.categoryBits = 0x0001;
      this.maskBits = 0xFFFF;
      this.groupIndex = 0;
   };
   FilterData.prototype.Copy = function () {
      var copy = new FilterData();
      copy.categoryBits = this.categoryBits;
      copy.maskBits = this.maskBits;
      copy.groupIndex = this.groupIndex;
      return copy;
   }
   Fixture.Fixture = function () {
      this.filter = new FilterData();
   };
   Fixture.prototype.GetType = function () {
      return this.shape.GetType();
   }
   Fixture.prototype.GetShape = function () {
      return this.shape;
   }
   Fixture.prototype.SetSensor = function (sensor) {
      if (this.isSensor == sensor) return;
      this.isSensor = sensor;
      if (this.body == null) return;
      var edge = this.body.GetContactList();
      while (edge) {
         var contact = edge.contact;
         var fixtureA = contact.GetFixtureA();
         var fixtureB = contact.GetFixtureB();
         if (fixtureA == this || fixtureB == this) contact.SetSensor(fixtureA.IsSensor() || fixtureB.IsSensor());
         edge = edge.next;
      }
   }
   Fixture.prototype.IsSensor = function () {
      return this.isSensor;
   }
   Fixture.prototype.SetFilterData = function (filter) {
      this.filter = filter.Copy();
      if (this.body) return;
      var edge = this.body.GetContactList();
      while (edge) {
         var contact = edge.contact;
         var fixtureA = contact.GetFixtureA();
         var fixtureB = contact.GetFixtureB();
         if (fixtureA == this || fixtureB == this) contact.FlagForFiltering();
         edge = edge.next;
      }
   }
   Fixture.prototype.GetFilterData = function () {
      return this.filter.Copy();
   }
   Fixture.prototype.GetBody = function () {
      return this.body;
   }
   Fixture.prototype.GetNext = function () {
      return this.next;
   }
   Fixture.prototype.GetUserData = function () {
      return this.userData;
   }
   Fixture.prototype.SetUserData = function (data) {
      this.userData = data;
   }
   Fixture.prototype.TestPoint = function (p) {
      return this.shape.TestPoint(this.body.GetTransform(), p);
   }
   Fixture.prototype.RayCast = function (output, input) {
      return this.shape.RayCast(output, input, this.body.GetTransform());
   }
   Fixture.prototype.GetMassData = function (massData) {
      if (massData === undefined) massData = null;
      if (massData == null) {
         massData = new MassData();
      }
      this.shape.ComputeMass(massData, this.density);
      return massData;
   }
   Fixture.prototype.SetDensity = function (density) {
      if (density === undefined) density = 0;
      this.density = density;
   }
   Fixture.prototype.GetDensity = function () {
      return this.density;
   }
   Fixture.prototype.GetFriction = function () {
      return this.friction;
   }
   Fixture.prototype.SetFriction = function (friction) {
      if (friction === undefined) friction = 0;
      this.friction = friction;
   }
   Fixture.prototype.GetRestitution = function () {
      return this.restitution;
   }
   Fixture.prototype.SetRestitution = function (restitution) {
      if (restitution === undefined) restitution = 0;
      this.restitution = restitution;
   }
   Fixture.prototype.GetAABB = function () {
      return this.aabb;
   }
   Fixture.prototype.Fixture = function () {
      this.aabb = new AABB();
      this.userData = null;
      this.body = null;
      this.next = null;
      this.shape = null;
      this.density = 0.0;
      this.friction = 0.0;
      this.restitution = 0.0;
   }
   Fixture.prototype.Create = function (body, xf, def) {
      this.userData = def.userData;
      this.friction = def.friction;
      this.restitution = def.restitution;
      this.body = body;
      this.next = null;
      this.filter = def.filter.Copy();
      this.isSensor = def.isSensor;
      this.shape = def.shape.Copy();
      this.density = def.density;
   }
   Fixture.prototype.Destroy = function () {
      this.shape = null;
   }
   Fixture.prototype.CreateProxy = function (broadPhase, xf) {
      this.shape.ComputeAABB(this.aabb, xf);
      this.proxy = broadPhase.CreateProxy(this.aabb, this);
   }
   Fixture.prototype.DestroyProxy = function (broadPhase) {
      if (this.proxy == null) {
         return;
      }
      broadPhase.DestroyProxy(this.proxy);
      this.proxy = null;
   }
   Fixture.prototype.Synchronize = function (broadPhase, transform1, transform2) {
      if (!this.proxy) return;
      var aabb1 = new AABB();
      var aab = new AABB();
      this.shape.ComputeAABB(aabb1, transform1);
      this.shape.ComputeAABB(aab, transform2);
      this.aabb.Combine(aabb1, aab);
      var displacement = Math.SubtractVV(transform2.position, transform1.position);
      broadPhase.MoveProxy(this.proxy, this.aabb, displacement);
   }
   FixtureDef.FixtureDef = function () {
      this.filter = new FilterData();
   };
   FixtureDef.prototype.FixtureDef = function () {
      this.shape = null;
      this.userData = null;
      this.friction = 0.2;
      this.restitution = 0.0;
      this.density = 0.0;
      this.filter.categoryBits = 0x0001;
      this.filter.maskBits = 0xFFFF;
      this.filter.groupIndex = 0;
      this.isSensor = false;
   }
   Island.Island = function () {};
   Island.prototype.Island = function () {
      this.bodies = new Vector();
      this.contacts = new Vector();
      this.joints = new Vector();
   }
   Island.prototype.Initialize = function (bodyCapacity, contactCapacity, jointCapacity, allocator, listener, contactSolver) {
      if (bodyCapacity === undefined) bodyCapacity = 0;
      if (contactCapacity === undefined) contactCapacity = 0;
      if (jointCapacity === undefined) jointCapacity = 0;
      var i = 0;
      this.bodyCapacity = bodyCapacity;
      this.contactCapacity = contactCapacity;
      this.jointCapacity = jointCapacity;
      this.bodyCount = 0;
      this.contactCount = 0;
      this.jointCount = 0;
      this.allocator = allocator;
      this.listener = listener;
      this.contactSolver = contactSolver;
      for (i = this.bodies.length;
      i < bodyCapacity; i++)
      this.bodies[i] = null;
      for (i = this.contacts.length;
      i < contactCapacity; i++)
      this.contacts[i] = null;
      for (i = this.joints.length;
      i < jointCapacity; i++)
      this.joints[i] = null;
   }
   Island.prototype.Clear = function () {
      this.bodyCount = 0;
      this.contactCount = 0;
      this.jointCount = 0;
   }
   Island.prototype.Solve = function (step, gravity, allowSleep) {
      var i = 0;
      var j = 0;
      var b;
      var joint;
      for (i = 0;
      i < this.bodyCount; ++i) {
         b = this.bodies[i];
         if (b.GetType() != Body._dynamicBody) continue;
         b.linearVelocity.x += step.dt * (gravity.x + b.invMass * b.force.x);
         b.linearVelocity.y += step.dt * (gravity.y + b.invMass * b.force.y);
         b.angularVelocity += step.dt * b.invI * b.torque;
         b.linearVelocity.Multiply(Math.Clamp(1.0 - step.dt * b.linearDamping, 0.0, 1.0));
         b.angularVelocity *= Math.Clamp(1.0 - step.dt * b.angularDamping, 0.0, 1.0);
      }
      this.contactSolver.Initialize(step, this.contacts, this.contactCount, this.allocator);
      var contactSolver = this.contactSolver;
      contactSolver.InitVelocityConstraints(step);
      for (i = 0;
      i < this.jointCount; ++i) {
         joint = this.joints[i];
         joint.InitVelocityConstraints(step);
      }
      for (i = 0;
      i < step.velocityIterations; ++i) {
         for (j = 0;
         j < this.jointCount; ++j) {
            joint = this.joints[j];
            joint.SolveVelocityConstraints(step);
         }
         contactSolver.SolveVelocityConstraints();
      }
      for (i = 0;
      i < this.jointCount; ++i) {
         joint = this.joints[i];
         joint.FinalizeVelocityConstraints();
      }
      contactSolver.FinalizeVelocityConstraints();
      for (i = 0;
      i < this.bodyCount; ++i) {
         b = this.bodies[i];
         if (b.GetType() == Body._staticBody) continue;
         var translationX = step.dt * b.linearVelocity.x;
         var translationY = step.dt * b.linearVelocity.y;
         if ((translationX * translationX + translationY * translationY) > Settings._maxTranslationSquared) {
            b.linearVelocity.Normalize();
            b.linearVelocity.x *= Settings._maxTranslation * step.inv_dt;
            b.linearVelocity.y *= Settings._maxTranslation * step.inv_dt;
         }
         var rotation = step.dt * b.angularVelocity;
         if (rotation * rotation > Settings._maxRotationSquared) {
            if (b.angularVelocity < 0.0) {
               b.angularVelocity = (-Settings._maxRotation * step.inv_dt);
            }
            else {
               b.angularVelocity = Settings._maxRotation * step.inv_dt;
            }
         }
         b.sweep.c0.SetV(b.sweep.c);
         b.sweep.a0 = b.sweep.a;
         b.sweep.c.x += step.dt * b.linearVelocity.x;
         b.sweep.c.y += step.dt * b.linearVelocity.y;
         b.sweep.a += step.dt * b.angularVelocity;
         b.SynchronizeTransform();
      }
      for (i = 0;
      i < step.positionIterations; ++i) {
         var contactsOkay = contactSolver.SolvePositionConstraints(Settings._contactBaumgarte);
         var jointsOkay = true;
         for (j = 0;
         j < this.jointCount; ++j) {
            joint = this.joints[j];
            var jointOkay = joint.SolvePositionConstraints(Settings._contactBaumgarte);
            jointsOkay = jointsOkay && jointOkay;
         }
         if (contactsOkay && jointsOkay) {
            break;
         }
      }
      this.Report(contactSolver.constraints);
      if (allowSleep) {
         var minSleepTime = Number.MAX_VALUE;
         var linTolSqr = Settings._linearSleepTolerance * Settings._linearSleepTolerance;
         var angTolSqr = Settings._angularSleepTolerance * Settings._angularSleepTolerance;
         for (i = 0;
         i < this.bodyCount; ++i) {
            b = this.bodies[i];
            if (b.GetType() == Body._staticBody) {
               continue;
            }
            if ((b.flags & Body.e_allowSleepFlag) == 0) {
               b.sleepTime = 0.0;
               minSleepTime = 0.0;
            }
            if ((b.flags & Body.e_allowSleepFlag) == 0 || b.angularVelocity * b.angularVelocity > angTolSqr || Math.Dot(b.linearVelocity, b.linearVelocity) > linTolSqr) {
               b.sleepTime = 0.0;
               minSleepTime = 0.0;
            }
            else {
               b.sleepTime += step.dt;
               minSleepTime = Math.Min(minSleepTime, b.sleepTime);
            }
         }
         if (minSleepTime >= Settings._timeToSleep) {
            for (i = 0;
            i < this.bodyCount; ++i) {
               b = this.bodies[i];
               b.SetAwake(false);
            }
         }
      }
   }
   Island.prototype.SolveTOI = function (subStep) {
      var i = 0;
      var j = 0;
      this.contactSolver.Initialize(subStep, this.contacts, this.contactCount, this.allocator);
      var contactSolver = this.contactSolver;
      for (i = 0;
      i < this.jointCount; ++i) {
         this.joints[i].InitVelocityConstraints(subStep);
      }
      for (i = 0;
      i < subStep.velocityIterations; ++i) {
         contactSolver.SolveVelocityConstraints();
         for (j = 0;
         j < this.jointCount; ++j) {
            this.joints[j].SolveVelocityConstraints(subStep);
         }
      }
      for (i = 0;
      i < this.bodyCount; ++i) {
         var b = this.bodies[i];
         if (b.GetType() == Body._staticBody) continue;
         var translationX = subStep.dt * b.linearVelocity.x;
         var translationY = subStep.dt * b.linearVelocity.y;
         if ((translationX * translationX + translationY * translationY) > Settings._maxTranslationSquared) {
            b.linearVelocity.Normalize();
            b.linearVelocity.x *= Settings._maxTranslation * subStep.inv_dt;
            b.linearVelocity.y *= Settings._maxTranslation * subStep.inv_dt;
         }
         var rotation = subStep.dt * b.angularVelocity;
         if (rotation * rotation > Settings._maxRotationSquared) {
            if (b.angularVelocity < 0.0) {
               b.angularVelocity = (-Settings._maxRotation * subStep.inv_dt);
            }
            else {
               b.angularVelocity = Settings._maxRotation * subStep.inv_dt;
            }
         }
         b.sweep.c0.SetV(b.sweep.c);
         b.sweep.a0 = b.sweep.a;
         b.sweep.c.x += subStep.dt * b.linearVelocity.x;
         b.sweep.c.y += subStep.dt * b.linearVelocity.y;
         b.sweep.a += subStep.dt * b.angularVelocity;
         b.SynchronizeTransform();
      }
      var k_toiBaumgarte = 0.75;
      for (i = 0;
      i < subStep.positionIterations; ++i) {
         var contactsOkay = contactSolver.SolvePositionConstraints(k_toiBaumgarte);
         var jointsOkay = true;
         for (j = 0;
         j < this.jointCount; ++j) {
            var jointOkay = this.joints[j].SolvePositionConstraints(Settings._contactBaumgarte);
            jointsOkay = jointsOkay && jointOkay;
         }
         if (contactsOkay && jointsOkay) {
            break;
         }
      }
      this.Report(contactSolver.constraints);
   }
   Island.prototype.Report = function (constraints) {
      if (this.listener == null) {
         return;
      }
      for (var i = 0; i < this.contactCount; ++i) {
         var c = this.contacts[i];
         var cc = constraints[i];
         for (var j = 0; j < cc.pointCount; ++j) {
            Island.s_impulse.normalImpulses[j] = cc.points[j].normalImpulse;
            Island.s_impulse.tangentImpulses[j] = cc.points[j].tangentImpulse;
         }
         this.listener.PostSolve(c, Island.s_impulse);
      }
   }
   Island.prototype.AddBody = function (body) {
      body.islandIndex = this.bodyCount;
      this.bodies[this.bodyCount++] = body;
   }
   Island.prototype.AddContact = function (contact) {
      this.contacts[this.contactCount++] = contact;
   }
   Island.prototype.AddJoint = function (joint) {
      this.joints[this.jointCount++] = joint;
   }
   Box2D.postDefs.push(function () {
      Box2D.Dynamics.Island.s_impulse = new ContactImpulse();
   });
   TimeStep.TimeStep = function () {};
   TimeStep.prototype.Set = function (step) {
      this.dt = step.dt;
      this.inv_dt = step.inv_dt;
      this.positionIterations = step.positionIterations;
      this.velocityIterations = step.velocityIterations;
      this.warmStarting = step.warmStarting;
   }
   World.World = function () {
      this.s_stack = new Vector();
      this.contactManager = new ContactManager();
      this.contactSolver = new ContactSolver();
      this.island = new Island();
   };
   World.prototype.World = function (gravity, doSleep) {
      this.destructionListener = null;
      this.debugDraw = null;
      this.bodyList = null;
      this.contactList = null;
      this.jointList = null;
      this.controllerList = null;
      this.bodyCount = 0;
      this.contactCount = 0;
      this.jointCount = 0;
      this.controllerCount = 0;
      World.warmStarting = true;
      World.continuousPhysics = true;
      this.allowSleep = doSleep;
      this.gravity = gravity;
      this.inv_dt0 = 0.0;
      this.contactManager.world = this;
      var bd = new BodyDef();
      this.groundBody = this.CreateBody(bd);
   }
   World.prototype.SetDestructionListener = function (listener) {
      this.destructionListener = listener;
   }
   World.prototype.SetContactFilter = function (filter) {
      this.contactManager.contactFilter = filter;
   }
   World.prototype.SetContactListener = function (listener) {
      this.contactManager.contactListener = listener;
   }
   World.prototype.SetDebugDraw = function (debugDraw) {
      this.debugDraw = debugDraw;
   }
   World.prototype.SetBroadPhase = function (broadPhase) {
      var oldBroadPhase = this.contactManager.broadPhase;
      this.contactManager.broadPhase = broadPhase;
      for (var b = this.bodyList; b; b = b.next) {
         for (var f = b.fixtureList; f; f = f.next) {
            f.proxy = broadPhase.CreateProxy(oldBroadPhase.GetFatAABB(f.proxy), f);
         }
      }
   }
   World.prototype.Validate = function () {
      this.contactManager.broadPhase.Validate();
   }
   World.prototype.GetProxyCount = function () {
      return this.contactManager.broadPhase.GetProxyCount();
   }
   World.prototype.CreateBody = function (def) {
      if (this.IsLocked() == true) {
         return null;
      }
      var b = new Body(def, this);
      b.prev = null;
      b.next = this.bodyList;
      if (this.bodyList) {
         this.bodyList.prev = b;
      }
      this.bodyList = b;
      ++this.bodyCount;
      return b;
   }
   World.prototype.DestroyBody = function (b) {
      if (this.IsLocked() == true) {
         return;
      }
      var jn = b.jointList;
      while (jn) {
         var jn0 = jn;
         jn = jn.next;
         if (this.destructionListener) {
            this.destructionListener.SayGoodbyeJoint(jn0.joint);
         }
         this.DestroyJoint(jn0.joint);
      }
      var coe = b.controllerList;
      while (coe) {
         var coe0 = coe;
         coe = coe.nextController;
         coe0.controller.RemoveBody(b);
      }
      var ce = b.contactList;
      while (ce) {
         var ce0 = ce;
         ce = ce.next;
         this.contactManager.Destroy(ce0.contact);
      }
      b.contactList = null;
      var f = b.fixtureList;
      while (f) {
         var f0 = f;
         f = f.next;
         if (this.destructionListener) {
            this.destructionListener.SayGoodbyeFixture(f0);
         }
         f0.DestroyProxy(this.contactManager.broadPhase);
         f0.Destroy();
      }
      b.fixtureList = null;
      b.fixtureCount = 0;
      if (b.prev) {
         b.prev.next = b.next;
      }
      if (b.next) {
         b.next.prev = b.prev;
      }
      if (b == this.bodyList) {
         this.bodyList = b.next;
      }--this.bodyCount;
   }
   World.prototype.CreateJoint = function (def) {
      var j = Joint.Create(def, null);
      j.prev = null;
      j.next = this.jointList;
      if (this.jointList) {
         this.jointList.prev = j;
      }
      this.jointList = j;
      ++this.jointCount;
      j.edgeA.joint = j;
      j.edgeA.other = j.bodyB;
      j.edgeA.prev = null;
      j.edgeA.next = j.bodyA.jointList;
      if (j.bodyA.jointList) j.bodyA.jointList.prev = j.edgeA;
      j.bodyA.jointList = j.edgeA;
      j.edgeB.joint = j;
      j.edgeB.other = j.bodyA;
      j.edgeB.prev = null;
      j.edgeB.next = j.bodyB.jointList;
      if (j.bodyB.jointList) j.bodyB.jointList.prev = j.edgeB;
      j.bodyB.jointList = j.edgeB;
      var bodyA = def.bodyA;
      var bodyB = def.bodyB;
      if (def.collideConnected == false) {
         var edge = bodyB.GetContactList();
         while (edge) {
            if (edge.other == bodyA) {
               edge.contact.FlagForFiltering();
            }
            edge = edge.next;
         }
      }
      return j;
   }
   World.prototype.DestroyJoint = function (j) {
      var collideConnected = j.collideConnected;
      if (j.prev) {
         j.prev.next = j.next;
      }
      if (j.next) {
         j.next.prev = j.prev;
      }
      if (j == this.jointList) {
         this.jointList = j.next;
      }
      var bodyA = j.bodyA;
      var bodyB = j.bodyB;
      bodyA.SetAwake(true);
      bodyB.SetAwake(true);
      if (j.edgeA.prev) {
         j.edgeA.prev.next = j.edgeA.next;
      }
      if (j.edgeA.next) {
         j.edgeA.next.prev = j.edgeA.prev;
      }
      if (j.edgeA == bodyA.jointList) {
         bodyA.jointList = j.edgeA.next;
      }
      j.edgeA.prev = null;
      j.edgeA.next = null;
      if (j.edgeB.prev) {
         j.edgeB.prev.next = j.edgeB.next;
      }
      if (j.edgeB.next) {
         j.edgeB.next.prev = j.edgeB.prev;
      }
      if (j.edgeB == bodyB.jointList) {
         bodyB.jointList = j.edgeB.next;
      }
      j.edgeB.prev = null;
      j.edgeB.next = null;
      Joint.Destroy(j, null);
      --this.jointCount;
      if (collideConnected == false) {
         var edge = bodyB.GetContactList();
         while (edge) {
            if (edge.other == bodyA) {
               edge.contact.FlagForFiltering();
            }
            edge = edge.next;
         }
      }
   }
   World.prototype.AddController = function (c) {
      c.next = this.controllerList;
      c.prev = null;
      this.controllerList = c;
      c.world = this;
      this.controllerCount++;
      return c;
   }
   World.prototype.RemoveController = function (c) {
      if (c.prev) c.prev.next = c.next;
      if (c.next) c.next.prev = c.prev;
      if (this.controllerList == c) this.controllerList = c.next;
      this.controllerCount--;
   }
   World.prototype.CreateController = function (controller) {
      if (controller.world != this) throw new Error("Controller can only be a member of one world");
      controller.next = this.controllerList;
      controller.prev = null;
      if (this.controllerList) this.controllerList.prev = controller;
      this.controllerList = controller;
      ++this.controllerCount;
      controller.world = this;
      return controller;
   }
   World.prototype.DestroyController = function (controller) {
      controller.Clear();
      if (controller.next) controller.next.prev = controller.prev;
      if (controller.prev) controller.prev.next = controller.next;
      if (controller == this.controllerList) this.controllerList = controller.next;
      --this.controllerCount;
   }
   World.prototype.SetWarmStarting = function (flag) {
      World.warmStarting = flag;
   }
   World.prototype.SetContinuousPhysics = function (flag) {
      World.continuousPhysics = flag;
   }
   World.prototype.GetBodyCount = function () {
      return this.bodyCount;
   }
   World.prototype.GetJointCount = function () {
      return this.jointCount;
   }
   World.prototype.GetContactCount = function () {
      return this.contactCount;
   }
   World.prototype.SetGravity = function (gravity) {
      this.gravity = gravity;
   }
   World.prototype.GetGravity = function () {
      return this.gravity;
   }
   World.prototype.GetGroundBody = function () {
      return this.groundBody;
   }
   World.prototype.Step = function (dt, velocityIterations, positionIterations) {
      if (dt === undefined) dt = 0;
      if (velocityIterations === undefined) velocityIterations = 0;
      if (positionIterations === undefined) positionIterations = 0;
      if (this.flags & World.e_newFixture) {
         this.contactManager.FindNewContacts();
         this.flags &= ~World.e_newFixture;
      }
      this.flags |= World.e_locked;
      var step = World.s_timestep2;
      step.dt = dt;
      step.velocityIterations = velocityIterations;
      step.positionIterations = positionIterations;
      if (dt > 0.0) {
         step.inv_dt = 1.0 / dt;
      }
      else {
         step.inv_dt = 0.0;
      }
      step.dtRatio = this.inv_dt0 * dt;
      step.warmStarting = World.warmStarting;
      this.contactManager.Collide();
      if (step.dt > 0.0) {
         this.Solve(step);
      }
      if (World.continuousPhysics && step.dt > 0.0) {
         this.SolveTOI(step);
      }
      if (step.dt > 0.0) {
         this.inv_dt0 = step.inv_dt;
      }
      this.flags &= ~World.e_locked;
   }
   World.prototype.ClearForces = function () {
      for (var body = this.bodyList; body; body = body.next) {
         body.force.SetZero();
         body.torque = 0.0;
      }
   }
   World.prototype.DrawDebugData = function () {
      if (this.debugDraw == null) {
         return;
      }
      this.debugDraw.sprite.graphics.clear();
      var flags = this.debugDraw.GetFlags();
      var i = 0;
      var b;
      var f;
      var s;
      var j;
      var bp;
      var invQ = new Vec2;
      var x1 = new Vec2;
      var x2 = new Vec2;
      var xf;
      var b1 = new AABB();
      var  = new AABB();
      var vs = [new Vec2(), new Vec2(), new Vec2(), new Vec2()];
      var color = new Color(0, 0, 0);
      if (flags & DebugDraw.e_shapeBit) {
         for (b = this.bodyList;
         b; b = b.next) {
            xf = b.xf;
            for (f = b.GetFixtureList();
            f; f = f.next) {
               s = f.GetShape();
               if (b.IsActive() == false) {
                  color.Set(0.5, 0.5, 0.3);
                  this.DrawShape(s, xf, color);
               }
               else if (b.GetType() == Body._staticBody) {
                  color.Set(0.5, 0.9, 0.5);
                  this.DrawShape(s, xf, color);
               }
               else if (b.GetType() == Body._kinematicBody) {
                  color.Set(0.5, 0.5, 0.9);
                  this.DrawShape(s, xf, color);
               }
               else if (b.IsAwake() == false) {
                  color.Set(0.6, 0.6, 0.6);
                  this.DrawShape(s, xf, color);
               }
               else {
                  color.Set(0.9, 0.7, 0.7);
                  this.DrawShape(s, xf, color);
               }
            }
         }
      }
      if (flags & DebugDraw.e_jointBit) {
         for (j = this.jointList;
         j; j = j.next) {
            this.DrawJoint(j);
         }
      }
      if (flags & DebugDraw.e_controllerBit) {
         for (var c = this.controllerList; c; c = c.next) {
            c.Draw(this.debugDraw);
         }
      }
      if (flags & DebugDraw.e_pairBit) {
         color.Set(0.3, 0.9, 0.9);
         for (var contact = this.contactManager.contactList; contact; contact = contact.GetNext()) {
            var fixtureA = contact.GetFixtureA();
            var fixtureB = contact.GetFixtureB();
            var cA = fixtureA.GetAABB().GetCenter();
            var cB = fixtureB.GetAABB().GetCenter();
            this.debugDraw.DrawSegment(cA, cB, color);
         }
      }
      if (flags & DebugDraw.e_aabbBit) {
         bp = this.contactManager.broadPhase;
         vs = [new Vec2(), new Vec2(), new Vec2(), new Vec2()];
         for (b = this.bodyList;
         b; b = b.GetNext()) {
            if (b.IsActive() == false) {
               continue;
            }
            for (f = b.GetFixtureList();
            f; f = f.GetNext()) {
               var aabb = bp.GetFatAABB(f.proxy);
               vs[0].Set(aabb.lowerBound.x, aabb.lowerBound.y);
               vs[1].Set(aabb.upperBound.x, aabb.lowerBound.y);
               vs[2].Set(aabb.upperBound.x, aabb.upperBound.y);
               vs[3].Set(aabb.lowerBound.x, aabb.upperBound.y);
               this.debugDraw.DrawPolygon(vs, 4, color);
            }
         }
      }
      if (flags & DebugDraw.e_centerOfMassBit) {
         for (b = this.bodyList;
         b; b = b.next) {
            xf = World.s_xf;
            xf.R = b.xf.R;
            xf.position = b.GetWorldCenter();
            this.debugDraw.DrawTransform(xf);
         }
      }
   }
   World.prototype.QueryAABB = function (callback, aabb) {
      var __this = this;
      var broadPhase = __this.contactManager.broadPhase;

      function WorldQueryWrapper(proxy) {
         return callback(broadPhase.GetUserData(proxy));
      };
      broadPhase.Query(WorldQueryWrapper, aabb);
   }
   World.prototype.QueryShape = function (callback, shape, transform) {
      var __this = this;
      if (transform === undefined) transform = null;
      if (transform == null) {
         transform = new Transform();
         transform.SetIdentity();
      }
      var broadPhase = __this.contactManager.broadPhase;

      function WorldQueryWrapper(proxy) {
         var fixture = (broadPhase.GetUserData(proxy) instanceof Fixture ? broadPhase.GetUserData(proxy) : null);
         if (Shape.TestOverlap(shape, transform, fixture.GetShape(), fixture.GetBody().GetTransform())) return callback(fixture);
         return true;
      };
      var aabb = new AABB();
      shape.ComputeAABB(aabb, transform);
      broadPhase.Query(WorldQueryWrapper, aabb);
   }
   World.prototype.QueryPoint = function (callback, p) {
      var __this = this;
      var broadPhase = __this.contactManager.broadPhase;

      function WorldQueryWrapper(proxy) {
         var fixture = (broadPhase.GetUserData(proxy) instanceof Fixture ? broadPhase.GetUserData(proxy) : null);
         if (fixture.TestPoint(p)) return callback(fixture);
         return true;
      };
      var aabb = new AABB();
      aabb.lowerBound.Set(p.x - Settings._linearSlop, p.y - Settings._linearSlop);
      aabb.upperBound.Set(p.x + Settings._linearSlop, p.y + Settings._linearSlop);
      broadPhase.Query(WorldQueryWrapper, aabb);
   }
   World.prototype.RayCast = function (callback, point1, point2) {
      var __this = this;
      var broadPhase = __this.contactManager.broadPhase;
      var output = new RayCastOutput;

      function RayCastWrapper(input, proxy) {
         var userData = broadPhase.GetUserData(proxy);
         var fixture = (userData instanceof Fixture ? userData : null);
         var hit = fixture.RayCast(output, input);
         if (hit) {
            var fraction = output.fraction;
            var point = new Vec2((1.0 - fraction) * point1.x + fraction * point2.x, (1.0 - fraction) * point1.y + fraction * point2.y);
            return callback(fixture, point, output.normal, fraction);
         }
         return input.maxFraction;
      };
      var input = new RayCastInput(point1, point2);
      broadPhase.RayCast(RayCastWrapper, input);
   }
   World.prototype.RayCastOne = function (point1, point2) {
      var __this = this;
      var result;

      function RayCastOneWrapper(fixture, point, normal, fraction) {
         if (fraction === undefined) fraction = 0;
         result = fixture;
         return fraction;
      };
      __this.RayCast(RayCastOneWrapper, point1, point2);
      return result;
   }
   World.prototype.RayCastAll = function (point1, point2) {
      var __this = this;
      var result = new Vector();

      function RayCastAllWrapper(fixture, point, normal, fraction) {
         if (fraction === undefined) fraction = 0;
         result[result.length] = fixture;
         return 1;
      };
      __this.RayCast(RayCastAllWrapper, point1, point2);
      return result;
   }
   World.prototype.GetBodyList = function () {
      return this.bodyList;
   }
   World.prototype.GetJointList = function () {
      return this.jointList;
   }
   World.prototype.GetContactList = function () {
      return this.contactList;
   }
   World.prototype.IsLocked = function () {
      return (this.flags & World.e_locked) > 0;
   }
   World.prototype.Solve = function (step) {
      var b;
      for (var controller = this.controllerList; controller; controller = controller.next) {
         controller.Step(step);
      }
      var island = this.island;
      island.Initialize(this.bodyCount, this.contactCount, this.jointCount, null, this.contactManager.contactListener, this.contactSolver);
      for (b = this.bodyList;
      b; b = b.next) {
         b.flags &= ~Body.e_islandFlag;
      }
      for (var c = this.contactList; c; c = c.next) {
         c.flags &= ~Contact.e_islandFlag;
      }
      for (var j = this.jointList; j; j = j.next) {
         j.islandFlag = false;
      }
      var stackSize = parseInt(this.bodyCount);
      var stack = this.s_stack;
      for (var seed = this.bodyList; seed; seed = seed.next) {
         if (seed.flags & Body.e_islandFlag) {
            continue;
         }
         if (seed.IsAwake() == false || seed.IsActive() == false) {
            continue;
         }
         if (seed.GetType() == Body._staticBody) {
            continue;
         }
         island.Clear();
         var stackCount = 0;
         stack[stackCount++] = seed;
         seed.flags |= Body.e_islandFlag;
         while (stackCount > 0) {
            b = stack[--stackCount];
            island.AddBody(b);
            if (b.IsAwake() == false) {
               b.SetAwake(true);
            }
            if (b.GetType() == Body._staticBody) {
               continue;
            }
            var other;
            for (var ce = b.contactList; ce; ce = ce.next) {
               if (ce.contact.flags & Contact.e_islandFlag) {
                  continue;
               }
               if (ce.contact.IsSensor() == true || ce.contact.IsEnabled() == false || ce.contact.IsTouching() == false) {
                  continue;
               }
               island.AddContact(ce.contact);
               ce.contact.flags |= Contact.e_islandFlag;
               other = ce.other;
               if (other.flags & Body.e_islandFlag) {
                  continue;
               }
               stack[stackCount++] = other;
               other.flags |= Body.e_islandFlag;
            }
            for (var jn = b.jointList; jn; jn = jn.next) {
               if (jn.joint.islandFlag == true) {
                  continue;
               }
               other = jn.other;
               if (other.IsActive() == false) {
                  continue;
               }
               island.AddJoint(jn.joint);
               jn.joint.islandFlag = true;
               if (other.flags & Body.e_islandFlag) {
                  continue;
               }
               stack[stackCount++] = other;
               other.flags |= Body.e_islandFlag;
            }
         }
         island.Solve(step, this.gravity, this.allowSleep);
         for (var i = 0; i < island.bodyCount; ++i) {
            b = island.bodies[i];
            if (b.GetType() == Body._staticBody) {
               b.flags &= ~Body.e_islandFlag;
            }
         }
      }
      for (i = 0;
      i < stack.length; ++i) {
         if (!stack[i]) break;
         stack[i] = null;
      }
      for (b = this.bodyList;
      b; b = b.next) {
         if (b.IsAwake() == false || b.IsActive() == false) {
            continue;
         }
         if (b.GetType() == Body._staticBody) {
            continue;
         }
         b.SynchronizeFixtures();
      }
      this.contactManager.FindNewContacts();
   }
   World.prototype.SolveTOI = function (step) {
      var b;
      var fA;
      var fB;
      var bA;
      var bB;
      var cEdge;
      var j;
      var island = this.island;
      island.Initialize(this.bodyCount, Settings._maxTOIContactsPerIsland, Settings._maxTOIJointsPerIsland, null, this.contactManager.contactListener, this.contactSolver);
      var queue = World.s_queue;
      for (b = this.bodyList;
      b; b = b.next) {
         b.flags &= ~Body.e_islandFlag;
         b.sweep.t0 = 0.0;
      }
      var c;
      for (c = this.contactList;
      c; c = c.next) {
         c.flags &= ~ (Contact.e_toiFlag | Contact.e_islandFlag);
      }
      for (j = this.jointList;
      j; j = j.next) {
         j.islandFlag = false;
      }
      for (;;) {
         var minContact = null;
         var minTOI = 1.0;
         for (c = this.contactList;
         c; c = c.next) {
            if (c.IsSensor() == true || c.IsEnabled() == false || c.IsContinuous() == false) {
               continue;
            }
            var toi = 1.0;
            if (c.flags & Contact.e_toiFlag) {
               toi = c.toi;
            }
            else {
               fA = c.fixtureA;
               fB = c.fixtureB;
               bA = fA.body;
               bB = fB.body;
               if ((bA.GetType() != Body._dynamicBody || bA.IsAwake() == false) && (bB.GetType() != Body._dynamicBody || bB.IsAwake() == false)) {
                  continue;
               }
               var t0 = bA.sweep.t0;
               if (bA.sweep.t0 < bB.sweep.t0) {
                  t0 = bB.sweep.t0;
                  bA.sweep.Advance(t0);
               }
               else if (bB.sweep.t0 < bA.sweep.t0) {
                  t0 = bA.sweep.t0;
                  bB.sweep.Advance(t0);
               }
               toi = c.ComputeTOI(bA.sweep, bB.sweep);
               Settings.Assert(0.0 <= toi && toi <= 1.0);
               if (toi > 0.0 && toi < 1.0) {
                  toi = (1.0 - toi) * t0 + toi;
                  if (toi > 1) toi = 1;
               }
               c.toi = toi;
               c.flags |= Contact.e_toiFlag;
            }
            if (Number.MIN_VALUE < toi && toi < minTOI) {
               minContact = c;
               minTOI = toi;
            }
         }
         if (minContact == null || 1.0 - 100.0 * Number.MIN_VALUE < minTOI) {
            break;
         }
         fA = minContact.fixtureA;
         fB = minContact.fixtureB;
         bA = fA.body;
         bB = fB.body;
         World.s_backupA.Set(bA.sweep);
         World.s_backupB.Set(bB.sweep);
         bA.Advance(minTOI);
         bB.Advance(minTOI);
         minContact.Update(this.contactManager.contactListener);
         minContact.flags &= ~Contact.e_toiFlag;
         if (minContact.IsSensor() == true || minContact.IsEnabled() == false) {
            bA.sweep.Set(World.s_backupA);
            bB.sweep.Set(World.s_backupB);
            bA.SynchronizeTransform();
            bB.SynchronizeTransform();
            continue;
         }
         if (minContact.IsTouching() == false) {
            continue;
         }
         var seed = bA;
         if (seed.GetType() != Body._dynamicBody) {
            seed = bB;
         }
         island.Clear();
         var queueStart = 0;
         var queueSize = 0;
         queue[queueStart + queueSize++] = seed;
         seed.flags |= Body.e_islandFlag;
         while (queueSize > 0) {
            b = queue[queueStart++];
            --queueSize;
            island.AddBody(b);
            if (b.IsAwake() == false) {
               b.SetAwake(true);
            }
            if (b.GetType() != Body._dynamicBody) {
               continue;
            }
            for (cEdge = b.contactList;
            cEdge; cEdge = cEdge.next) {
               if (island.contactCount == island.contactCapacity) {
                  break;
               }
               if (cEdge.contact.flags & Contact.e_islandFlag) {
                  continue;
               }
               if (cEdge.contact.IsSensor() == true || cEdge.contact.IsEnabled() == false || cEdge.contact.IsTouching() == false) {
                  continue;
               }
               island.AddContact(cEdge.contact);
               cEdge.contact.flags |= Contact.e_islandFlag;
               var other = cEdge.other;
               if (other.flags & Body.e_islandFlag) {
                  continue;
               }
               if (other.GetType() != Body._staticBody) {
                  other.Advance(minTOI);
                  other.SetAwake(true);
               }
               queue[queueStart + queueSize] = other;
               ++queueSize;
               other.flags |= Body.e_islandFlag;
            }
            for (var jEdge = b.jointList; jEdge; jEdge = jEdge.next) {
               if (island.jointCount == island.jointCapacity) continue;
               if (jEdge.joint.islandFlag == true) continue;
               other = jEdge.other;
               if (other.IsActive() == false) {
                  continue;
               }
               island.AddJoint(jEdge.joint);
               jEdge.joint.islandFlag = true;
               if (other.flags & Body.e_islandFlag) continue;
               if (other.GetType() != Body._staticBody) {
                  other.Advance(minTOI);
                  other.SetAwake(true);
               }
               queue[queueStart + queueSize] = other;
               ++queueSize;
               other.flags |= Body.e_islandFlag;
            }
         }
         var subStep = World.s_timestep;
         subStep.warmStarting = false;
         subStep.dt = (1.0 - minTOI) * step.dt;
         subStep.inv_dt = 1.0 / subStep.dt;
         subStep.dtRatio = 0.0;
         subStep.velocityIterations = step.velocityIterations;
         subStep.positionIterations = step.positionIterations;
         island.SolveTOI(subStep);
         var i = 0;
         for (i = 0;
         i < island.bodyCount; ++i) {
            b = island.bodies[i];
            b.flags &= ~Body.e_islandFlag;
            if (b.IsAwake() == false) {
               continue;
            }
            if (b.GetType() != Body._dynamicBody) {
               continue;
            }
            b.SynchronizeFixtures();
            for (cEdge = b.contactList;
            cEdge; cEdge = cEdge.next) {
               cEdge.contact.flags &= ~Contact.e_toiFlag;
            }
         }
         for (i = 0;
         i < island.contactCount; ++i) {
            c = island.contacts[i];
            c.flags &= ~ (Contact.e_toiFlag | Contact.e_islandFlag);
         }
         for (i = 0;
         i < island.jointCount; ++i) {
            j = island.joints[i];
            j.islandFlag = false;
         }
         this.contactManager.FindNewContacts();
      }
   }
   World.prototype.DrawJoint = function (joint) {
      var b1 = joint.GetBodyA();
      var  = joint.GetBodyB();
      var xf1 = b1.xf;
      var xf2 = .xf;
      var x1 = xf1.position;
      var x2 = xf2.position;
      var p1 = joint.GetAnchorA();
      var p2 = joint.GetAnchorB();
      var color = World.s_jointColor;
      switch (joint.type) {
      case Joint.e_distanceJoint:
         this.debugDraw.DrawSegment(p1, p2, color);
         break;
      case Joint.e_pulleyJoint:
         {
            var pulley = ((joint instanceof PulleyJoint ? joint : null));
            var s1 = pulley.GetGroundAnchorA();
            var s2 = pulley.GetGroundAnchorB();
            this.debugDraw.DrawSegment(s1, p1, color);
            this.debugDraw.DrawSegment(s2, p2, color);
            this.debugDraw.DrawSegment(s1, s2, color);
         }
         break;
      case Joint.e_mouseJoint:
         this.debugDraw.DrawSegment(p1, p2, color);
         break;
      default:
         if (b1 != this.groundBody) this.debugDraw.DrawSegment(x1, p1, color);
         this.debugDraw.DrawSegment(p1, p2, color);
         if ( != this.groundBody) this.debugDraw.DrawSegment(x2, p2, color);
      }
   }
   World.prototype.DrawShape = function (shape, xf, color) {
      switch (shape.type) {
      case Shape.e_circleShape:
         {
            var circle = ((shape instanceof CircleShape ? shape : null));
            var center = Math.MulX(xf, circle.p);
            var radius = circle.radius;
            var axis = xf.R.col1;
            this.debugDraw.DrawSolidCircle(center, radius, axis, color);
         }
         break;
      case Shape.e_polygonShape:
         {
            var i = 0;
            var poly = ((shape instanceof PolygonShape ? shape : null));
            var vertexCount = parseInt(poly.GetVertexCount());
            var localVertices = poly.GetVertices();
            var vertices = new Vector(vertexCount);
            for (i = 0;
            i < vertexCount; ++i) {
               vertices[i] = Math.MulX(xf, localVertices[i]);
            }
            this.debugDraw.DrawSolidPolygon(vertices, vertexCount, color);
         }
         break;
      case Shape.e_edgeShape:
         {
            var edge = (shape instanceof EdgeShape ? shape : null);
            this.debugDraw.DrawSegment(Math.MulX(xf, edge.GetVertex1()), Math.MulX(xf, edge.GetVertex2()), color);
         }
         break;
      }
   }
   Box2D.postDefs.push(function () {
      Box2D.Dynamics.World.s_timestep2 = new TimeStep();
      Box2D.Dynamics.World.s_xf = new Transform();
      Box2D.Dynamics.World.s_backupA = new Sweep();
      Box2D.Dynamics.World.s_backupB = new Sweep();
      Box2D.Dynamics.World.s_timestep = new TimeStep();
      Box2D.Dynamics.World.s_queue = new Vector();
      Box2D.Dynamics.World.s_jointColor = new Color(0.5, 0.8, 0.8);
      Box2D.Dynamics.World.e_newFixture = 0x0001;
      Box2D.Dynamics.World.e_locked = 0x0002;
   });
})();
(function () {
   var CircleShape = Box2D.Collision.Shapes.CircleShape,
      EdgeChainDef = Box2D.Collision.Shapes.EdgeChainDef,
      EdgeShape = Box2D.Collision.Shapes.EdgeShape,
      MassData = Box2D.Collision.Shapes.MassData,
      PolygonShape = Box2D.Collision.Shapes.PolygonShape,
      Shape = Box2D.Collision.Shapes.Shape,
      CircleContact = Box2D.Dynamics.Contacts.CircleContact,
      Contact = Box2D.Dynamics.Contacts.Contact,
      ContactConstraint = Box2D.Dynamics.Contacts.ContactConstraint,
      ContactConstraintPoint = Box2D.Dynamics.Contacts.ContactConstraintPoint,
      ContactEdge = Box2D.Dynamics.Contacts.ContactEdge,
      ContactFactory = Box2D.Dynamics.Contacts.ContactFactory,
      ContactRegister = Box2D.Dynamics.Contacts.ContactRegister,
      ContactResult = Box2D.Dynamics.Contacts.ContactResult,
      ContactSolver = Box2D.Dynamics.Contacts.ContactSolver,
      EdgeAndCircleContact = Box2D.Dynamics.Contacts.EdgeAndCircleContact,
      NullContact = Box2D.Dynamics.Contacts.NullContact,
      PolyAndCircleContact = Box2D.Dynamics.Contacts.PolyAndCircleContact,
      PolyAndEdgeContact = Box2D.Dynamics.Contacts.PolyAndEdgeContact,
      PolygonContact = Box2D.Dynamics.Contacts.PolygonContact,
      PositionSolverManifold = Box2D.Dynamics.Contacts.PositionSolverManifold,
      Body = Box2D.Dynamics.Body,
      BodyDef = Box2D.Dynamics.BodyDef,
      ContactFilter = Box2D.Dynamics.ContactFilter,
      ContactImpulse = Box2D.Dynamics.ContactImpulse,
      ContactListener = Box2D.Dynamics.ContactListener,
      ContactManager = Box2D.Dynamics.ContactManager,
      DebugDraw = Box2D.Dynamics.DebugDraw,
      DestructionListener = Box2D.Dynamics.DestructionListener,
      FilterData = Box2D.Dynamics.FilterData,
      Fixture = Box2D.Dynamics.Fixture,
      FixtureDef = Box2D.Dynamics.FixtureDef,
      Island = Box2D.Dynamics.Island,
      TimeStep = Box2D.Dynamics.TimeStep,
      World = Box2D.Dynamics.World,
      Color = Box2D.Common.Color,
      internal = Box2D.Common.internal,
      Settings = Box2D.Common.Settings,
      Mat22 = Box2D.Common.Math.Mat22,
      Mat33 = Box2D.Common.Math.Mat33,
      Math = Box2D.Common.Math.Math,
      Sweep = Box2D.Common.Math.Sweep,
      Transform = Box2D.Common.Math.Transform,
      Vec2 = Box2D.Common.Math.Vec2,
      Vec3 = Box2D.Common.Math.Vec3,
      AABB = Box2D.Collision.AABB,
      Bound = Box2D.Collision.Bound,
      BoundValues = Box2D.Collision.BoundValues,
      Collision = Box2D.Collision.Collision,
      ContactID = Box2D.Collision.ContactID,
      ContactPoint = Box2D.Collision.ContactPoint,
      Distance = Box2D.Collision.Distance,
      DistanceInput = Box2D.Collision.DistanceInput,
      DistanceOutput = Box2D.Collision.DistanceOutput,
      DistanceProxy = Box2D.Collision.DistanceProxy,
      DynamicTree = Box2D.Collision.DynamicTree,
      DynamicTreeBroadPhase = Box2D.Collision.DynamicTreeBroadPhase,
      DynamicTreeNode = Box2D.Collision.DynamicTreeNode,
      DynamicTreePair = Box2D.Collision.DynamicTreePair,
      Manifold = Box2D.Collision.Manifold,
      ManifoldPoint = Box2D.Collision.ManifoldPoint,
      Point = Box2D.Collision.Point,
      RayCastInput = Box2D.Collision.RayCastInput,
      RayCastOutput = Box2D.Collision.RayCastOutput,
      Segment = Box2D.Collision.Segment,
      SeparationFunction = Box2D.Collision.SeparationFunction,
      Simplex = Box2D.Collision.Simplex,
      SimplexCache = Box2D.Collision.SimplexCache,
      SimplexVertex = Box2D.Collision.SimplexVertex,
      TimeOfImpact = Box2D.Collision.TimeOfImpact,
      TOIInput = Box2D.Collision.TOIInput,
      WorldManifold = Box2D.Collision.WorldManifold,
      ClipVertex = Box2D.Collision.ClipVertex,
      Features = Box2D.Collision.Features,
      IBroadPhase = Box2D.Collision.IBroadPhase;

   Box2D.inherit(CircleContact, Box2D.Dynamics.Contacts.Contact);
   CircleContact.prototype.__super = Box2D.Dynamics.Contacts.Contact.prototype;
   CircleContact.CircleContact = function () {
      Box2D.Dynamics.Contacts.Contact.Contact.apply(this, arguments);
   };
   CircleContact.Create = function (allocator) {
      return new CircleContact();
   }
   CircleContact.Destroy = function (contact, allocator) {}
   CircleContact.prototype.Reset = function (fixtureA, fixtureB) {
      this.__super.Reset.call(this, fixtureA, fixtureB);
   }
   CircleContact.prototype.Evaluate = function () {
      var bA = this.fixtureA.GetBody();
      var bB = this.fixtureB.GetBody();
      Collision.CollideCircles(this.manifold, (this.fixtureA.GetShape() instanceof CircleShape ? this.fixtureA.GetShape() : null), bA.xf, (this.fixtureB.GetShape() instanceof CircleShape ? this.fixtureB.GetShape() : null), bB.xf);
   }
   Contact.Contact = function () {
      this.nodeA = new ContactEdge();
      this.nodeB = new ContactEdge();
      this.manifold = new Manifold();
      this.oldManifold = new Manifold();
   };
   Contact.prototype.GetManifold = function () {
      return this.manifold;
   }
   Contact.prototype.GetWorldManifold = function (worldManifold) {
      var bodyA = this.fixtureA.GetBody();
      var bodyB = this.fixtureB.GetBody();
      var shapeA = this.fixtureA.GetShape();
      var shapeB = this.fixtureB.GetShape();
      worldManifold.Initialize(this.manifold, bodyA.GetTransform(), shapeA.radius, bodyB.GetTransform(), shapeB.radius);
   }
   Contact.prototype.IsTouching = function () {
      return (this.flags & Contact.e_touchingFlag) == Contact.e_touchingFlag;
   }
   Contact.prototype.IsContinuous = function () {
      return (this.flags & Contact.e_continuousFlag) == Contact.e_continuousFlag;
   }
   Contact.prototype.SetSensor = function (sensor) {
      if (sensor) {
         this.flags |= Contact.e_sensorFlag;
      }
      else {
         this.flags &= ~Contact.e_sensorFlag;
      }
   }
   Contact.prototype.IsSensor = function () {
      return (this.flags & Contact.e_sensorFlag) == Contact.e_sensorFlag;
   }
   Contact.prototype.SetEnabled = function (flag) {
      if (flag) {
         this.flags |= Contact.e_enabledFlag;
      }
      else {
         this.flags &= ~Contact.e_enabledFlag;
      }
   }
   Contact.prototype.IsEnabled = function () {
      return (this.flags & Contact.e_enabledFlag) == Contact.e_enabledFlag;
   }
   Contact.prototype.GetNext = function () {
      return this.next;
   }
   Contact.prototype.GetFixtureA = function () {
      return this.fixtureA;
   }
   Contact.prototype.GetFixtureB = function () {
      return this.fixtureB;
   }
   Contact.prototype.FlagForFiltering = function () {
      this.flags |= Contact.e_filterFlag;
   }
   Contact.prototype.Contact = function () {}
   Contact.prototype.Reset = function (fixtureA, fixtureB) {
      if (fixtureA === undefined) fixtureA = null;
      if (fixtureB === undefined) fixtureB = null;
      this.flags = Contact.e_enabledFlag;
      if (!fixtureA || !fixtureB) {
         this.fixtureA = null;
         this.fixtureB = null;
         return;
      }
      if (fixtureA.IsSensor() || fixtureB.IsSensor()) {
         this.flags |= Contact.e_sensorFlag;
      }
      var bodyA = fixtureA.GetBody();
      var bodyB = fixtureB.GetBody();
      if (bodyA.GetType() != Body._dynamicBody || bodyA.IsBullet() || bodyB.GetType() != Body._dynamicBody || bodyB.IsBullet()) {
         this.flags |= Contact.e_continuousFlag;
      }
      this.fixtureA = fixtureA;
      this.fixtureB = fixtureB;
      this.manifold.pointCount = 0;
      this.prev = null;
      this.next = null;
      this.nodeA.contact = null;
      this.nodeA.prev = null;
      this.nodeA.next = null;
      this.nodeA.other = null;
      this.nodeB.contact = null;
      this.nodeB.prev = null;
      this.nodeB.next = null;
      this.nodeB.other = null;
   }
   Contact.prototype.Update = function (listener) {
      var tManifold = this.oldManifold;
      this.oldManifold = this.manifold;
      this.manifold = tManifold;
      this.flags |= Contact.e_enabledFlag;
      var touching = false;
      var wasTouching = (this.flags & Contact.e_touchingFlag) == Contact.e_touchingFlag;
      var bodyA = this.fixtureA.body;
      var bodyB = this.fixtureB.body;
      var aabbOverlap = this.fixtureA.aabb.TestOverlap(this.fixtureB.aabb);
      if (this.flags & Contact.e_sensorFlag) {
         if (aabbOverlap) {
            var shapeA = this.fixtureA.GetShape();
            var shapeB = this.fixtureB.GetShape();
            var xfA = bodyA.GetTransform();
            var xfB = bodyB.GetTransform();
            touching = Shape.TestOverlap(shapeA, xfA, shapeB, xfB);
         }
         this.manifold.pointCount = 0;
      }
      else {
         if (bodyA.GetType() != Body._dynamicBody || bodyA.IsBullet() || bodyB.GetType() != Body._dynamicBody || bodyB.IsBullet()) {
            this.flags |= Contact.e_continuousFlag;
         }
         else {
            this.flags &= ~Contact.e_continuousFlag;
         }
         if (aabbOverlap) {
            this.Evaluate();
            touching = this.manifold.pointCount > 0;
            for (var i = 0; i < this.manifold.pointCount; ++i) {
               var mp2 = this.manifold.points[i];
               mp2.normalImpulse = 0.0;
               mp2.tangentImpulse = 0.0;
               var id2 = mp2.id;
               for (var j = 0; j < this.oldManifold.pointCount; ++j) {
                  var mp1 = this.oldManifold.points[j];
                  if (mp1.id.key == id2.key) {
                     mp2.normalImpulse = mp1.normalImpulse;
                     mp2.tangentImpulse = mp1.tangentImpulse;
                     break;
                  }
               }
            }
         }
         else {
            this.manifold.pointCount = 0;
         }
         if (touching != wasTouching) {
            bodyA.SetAwake(true);
            bodyB.SetAwake(true);
         }
      }
      if (touching) {
         this.flags |= Contact.e_touchingFlag;
      }
      else {
         this.flags &= ~Contact.e_touchingFlag;
      }
      if (wasTouching == false && touching == true) {
         listener.BeginContact(this);
      }
      if (wasTouching == true && touching == false) {
         listener.EndContact(this);
      }
      if ((this.flags & Contact.e_sensorFlag) == 0) {
         listener.PreSolve(this, this.oldManifold);
      }
   }
   Contact.prototype.Evaluate = function () {}
   Contact.prototype.ComputeTOI = function (sweepA, sweepB) {
      Contact.s_input.proxyA.Set(this.fixtureA.GetShape());
      Contact.s_input.proxyB.Set(this.fixtureB.GetShape());
      Contact.s_input.sweepA = sweepA;
      Contact.s_input.sweepB = sweepB;
      Contact.s_input.tolerance = Settings._linearSlop;
      return TimeOfImpact.TimeOfImpact(Contact.s_input);
   }
   Box2D.postDefs.push(function () {
      Box2D.Dynamics.Contacts.Contact.e_sensorFlag = 0x0001;
      Box2D.Dynamics.Contacts.Contact.e_continuousFlag = 0x0002;
      Box2D.Dynamics.Contacts.Contact.e_islandFlag = 0x0004;
      Box2D.Dynamics.Contacts.Contact.e_toiFlag = 0x0008;
      Box2D.Dynamics.Contacts.Contact.e_touchingFlag = 0x0010;
      Box2D.Dynamics.Contacts.Contact.e_enabledFlag = 0x0020;
      Box2D.Dynamics.Contacts.Contact.e_filterFlag = 0x0040;
      Box2D.Dynamics.Contacts.Contact.s_input = new TOIInput();
   });
   ContactConstraint.ContactConstraint = function () {
      this.localPlaneNormal = new Vec2();
      this.localPoint = new Vec2();
      this.normal = new Vec2();
      this.normalMass = new Mat22();
      this.K = new Mat22();
   };
   ContactConstraint.prototype.ContactConstraint = function () {
      this.points = new Vector(Settings._maxManifoldPoints);
      for (var i = 0; i < Settings._maxManifoldPoints; i++) {
         this.points[i] = new ContactConstraintPoint();
      }
   }
   ContactConstraintPoint.ContactConstraintPoint = function () {
      this.localPoint = new Vec2();
      this.rA = new Vec2();
      this.rB = new Vec2();
   };
   ContactEdge.ContactEdge = function () {};
   ContactFactory.ContactFactory = function () {};
   ContactFactory.prototype.ContactFactory = function (allocator) {
      this.allocator = allocator;
      this.InitializeRegisters();
   }
   ContactFactory.prototype.AddType = function (createFcn, destroyFcn, type1, type2) {
      if (type1 === undefined) type1 = 0;
      if (type2 === undefined) type2 = 0;
      this.registers[type1][type2].createFcn = createFcn;
      this.registers[type1][type2].destroyFcn = destroyFcn;
      this.registers[type1][type2].primary = true;
      if (type1 != type2) {
         this.registers[type2][type1].createFcn = createFcn;
         this.registers[type2][type1].destroyFcn = destroyFcn;
         this.registers[type2][type1].primary = false;
      }
   }
   ContactFactory.prototype.InitializeRegisters = function () {
      this.registers = new Vector(Shape.e_shapeTypeCount);
      for (var i = 0; i < Shape.e_shapeTypeCount; i++) {
         this.registers[i] = new Vector(Shape.e_shapeTypeCount);
         for (var j = 0; j < Shape.e_shapeTypeCount; j++) {
            this.registers[i][j] = new ContactRegister();
         }
      }
      this.AddType(CircleContact.Create, CircleContact.Destroy, Shape.e_circleShape, Shape.e_circleShape);
      this.AddType(PolyAndCircleContact.Create, PolyAndCircleContact.Destroy, Shape.e_polygonShape, Shape.e_circleShape);
      this.AddType(PolygonContact.Create, PolygonContact.Destroy, Shape.e_polygonShape, Shape.e_polygonShape);
      this.AddType(EdgeAndCircleContact.Create, EdgeAndCircleContact.Destroy, Shape.e_edgeShape, Shape.e_circleShape);
      this.AddType(PolyAndEdgeContact.Create, PolyAndEdgeContact.Destroy, Shape.e_polygonShape, Shape.e_edgeShape);
   }
   ContactFactory.prototype.Create = function (fixtureA, fixtureB) {
      var type1 = parseInt(fixtureA.GetType());
      var type2 = parseInt(fixtureB.GetType());
      var reg = this.registers[type1][type2];
      var c;
      if (reg.pool) {
         c = reg.pool;
         reg.pool = c.next;
         reg.poolCount--;
         c.Reset(fixtureA, fixtureB);
         return c;
      }
      var createFcn = reg.createFcn;
      if (createFcn != null) {
         if (reg.primary) {
            c = createFcn(this.allocator);
            c.Reset(fixtureA, fixtureB);
            return c;
         }
         else {
            c = createFcn(this.allocator);
            c.Reset(fixtureB, fixtureA);
            return c;
         }
      }
      else {
         return null;
      }
   }
   ContactFactory.prototype.Destroy = function (contact) {
      if (contact.manifold.pointCount > 0) {
         contact.fixtureA.body.SetAwake(true);
         contact.fixtureB.body.SetAwake(true);
      }
      var type1 = parseInt(contact.fixtureA.GetType());
      var type2 = parseInt(contact.fixtureB.GetType());
      var reg = this.registers[type1][type2];
      if (true) {
         reg.poolCount++;
         contact.next = reg.pool;
         reg.pool = contact;
      }
      var destroyFcn = reg.destroyFcn;
      destroyFcn(contact, this.allocator);
   }
   ContactRegister.ContactRegister = function () {};
   ContactResult.ContactResult = function () {
      this.position = new Vec2();
      this.normal = new Vec2();
      this.id = new ContactID();
   };
   ContactSolver.ContactSolver = function () {
      this.step = new TimeStep();
      this.constraints = new Vector();
   };
   ContactSolver.prototype.ContactSolver = function () {}
   ContactSolver.prototype.Initialize = function (step, contacts, contactCount, allocator) {
      if (contactCount === undefined) contactCount = 0;
      var contact;
      this.step.Set(step);
      this.allocator = allocator;
      var i = 0;
      var tVec;
      var tMat;
      this.constraintCount = contactCount;
      while (this.constraints.length < this.constraintCount) {
         this.constraints[this.constraints.length] = new ContactConstraint();
      }
      for (i = 0;
      i < contactCount; ++i) {
         contact = contacts[i];
         var fixtureA = contact.fixtureA;
         var fixtureB = contact.fixtureB;
         var shapeA = fixtureA.shape;
         var shapeB = fixtureB.shape;
         var radiusA = shapeA.radius;
         var radiusB = shapeB.radius;
         var bodyA = fixtureA.body;
         var bodyB = fixtureB.body;
         var manifold = contact.GetManifold();
         var friction = Settings.MixFriction(fixtureA.GetFriction(), fixtureB.GetFriction());
         var restitution = Settings.MixRestitution(fixtureA.GetRestitution(), fixtureB.GetRestitution());
         var vAX = bodyA.linearVelocity.x;
         var vAY = bodyA.linearVelocity.y;
         var vBX = bodyB.linearVelocity.x;
         var vBY = bodyB.linearVelocity.y;
         var wA = bodyA.angularVelocity;
         var wB = bodyB.angularVelocity;
         Settings.Assert(manifold.pointCount > 0);
         ContactSolver.s_worldManifold.Initialize(manifold, bodyA.xf, radiusA, bodyB.xf, radiusB);
         var normalX = ContactSolver.s_worldManifold.normal.x;
         var normalY = ContactSolver.s_worldManifold.normal.y;
         var cc = this.constraints[i];
         cc.bodyA = bodyA;
         cc.bodyB = bodyB;
         cc.manifold = manifold;
         cc.normal.x = normalX;
         cc.normal.y = normalY;
         cc.pointCount = manifold.pointCount;
         cc.friction = friction;
         cc.restitution = restitution;
         cc.localPlaneNormal.x = manifold.localPlaneNormal.x;
         cc.localPlaneNormal.y = manifold.localPlaneNormal.y;
         cc.localPoint.x = manifold.localPoint.x;
         cc.localPoint.y = manifold.localPoint.y;
         cc.radius = radiusA + radiusB;
         cc.type = manifold.type;
         for (var k = 0; k < cc.pointCount; ++k) {
            var cp = manifold.points[k];
            var ccp = cc.points[k];
            ccp.normalImpulse = cp.normalImpulse;
            ccp.tangentImpulse = cp.tangentImpulse;
            ccp.localPoint.SetV(cp.localPoint);
            var rAX = ccp.rA.x = ContactSolver.s_worldManifold.points[k].x - bodyA.sweep.c.x;
            var rAY = ccp.rA.y = ContactSolver.s_worldManifold.points[k].y - bodyA.sweep.c.y;
            var rBX = ccp.rB.x = ContactSolver.s_worldManifold.points[k].x - bodyB.sweep.c.x;
            var rBY = ccp.rB.y = ContactSolver.s_worldManifold.points[k].y - bodyB.sweep.c.y;
            var rnA = rAX * normalY - rAY * normalX;
            var rnB = rBX * normalY - rBY * normalX;
            rnA *= rnA;
            rnB *= rnB;
            var kNormal = bodyA.invMass + bodyB.invMass + bodyA.invI * rnA + bodyB.invI * rnB;
            ccp.normalMass = 1.0 / kNormal;
            var kEqualized = bodyA.mass * bodyA.invMass + bodyB.mass * bodyB.invMass;
            kEqualized += bodyA.mass * bodyA.invI * rnA + bodyB.mass * bodyB.invI * rnB;
            ccp.equalizedMass = 1.0 / kEqualized;
            var tangentX = normalY;
            var tangentY = (-normalX);
            var rtA = rAX * tangentY - rAY * tangentX;
            var rtB = rBX * tangentY - rBY * tangentX;
            rtA *= rtA;
            rtB *= rtB;
            var kTangent = bodyA.invMass + bodyB.invMass + bodyA.invI * rtA + bodyB.invI * rtB;
            ccp.tangentMass = 1.0 / kTangent;
            ccp.velocityBias = 0.0;
            var tX = vBX + ((-wB * rBY)) - vAX - ((-wA * rAY));
            var tY = vBY + (wB * rBX) - vAY - (wA * rAX);
            var vRel = cc.normal.x * tX + cc.normal.y * tY;
            if (vRel < (-Settings._velocityThreshold)) {
               ccp.velocityBias += (-cc.restitution * vRel);
            }
         }
         if (cc.pointCount == 2) {
            var ccp1 = cc.points[0];
            var ccp2 = cc.points[1];
            var invMassA = bodyA.invMass;
            var invIA = bodyA.invI;
            var invMassB = bodyB.invMass;
            var invIB = bodyB.invI;
            var rn1A = ccp1.rA.x * normalY - ccp1.rA.y * normalX;
            var rn1B = ccp1.rB.x * normalY - ccp1.rB.y * normalX;
            var rn2A = ccp2.rA.x * normalY - ccp2.rA.y * normalX;
            var rn2B = ccp2.rB.x * normalY - ccp2.rB.y * normalX;
            var k11 = invMassA + invMassB + invIA * rn1A * rn1A + invIB * rn1B * rn1B;
            var k22 = invMassA + invMassB + invIA * rn2A * rn2A + invIB * rn2B * rn2B;
            var k12 = invMassA + invMassB + invIA * rn1A * rn2A + invIB * rn1B * rn2B;
            var k_maxConditionNumber = 100.0;
            if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12)) {
               cc.K.col1.Set(k11, k12);
               cc.K.col2.Set(k12, k22);
               cc.K.GetInverse(cc.normalMass);
            }
            else {
               cc.pointCount = 1;
            }
         }
      }
   }
   ContactSolver.prototype.InitVelocityConstraints = function (step) {
      var tVec;
      var tVec2;
      var tMat;
      for (var i = 0; i < this.constraintCount; ++i) {
         var c = this.constraints[i];
         var bodyA = c.bodyA;
         var bodyB = c.bodyB;
         var invMassA = bodyA.invMass;
         var invIA = bodyA.invI;
         var invMassB = bodyB.invMass;
         var invIB = bodyB.invI;
         var normalX = c.normal.x;
         var normalY = c.normal.y;
         var tangentX = normalY;
         var tangentY = (-normalX);
         var tX = 0;
         var j = 0;
         var tCount = 0;
         if (step.warmStarting) {
            tCount = c.pointCount;
            for (j = 0;
            j < tCount; ++j) {
               var ccp = c.points[j];
               ccp.normalImpulse *= step.dtRatio;
               ccp.tangentImpulse *= step.dtRatio;
               var PX = ccp.normalImpulse * normalX + ccp.tangentImpulse * tangentX;
               var PY = ccp.normalImpulse * normalY + ccp.tangentImpulse * tangentY;
               bodyA.angularVelocity -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
               bodyA.linearVelocity.x -= invMassA * PX;
               bodyA.linearVelocity.y -= invMassA * PY;
               bodyB.angularVelocity += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
               bodyB.linearVelocity.x += invMassB * PX;
               bodyB.linearVelocity.y += invMassB * PY;
            }
         }
         else {
            tCount = c.pointCount;
            for (j = 0;
            j < tCount; ++j) {
               var ccp2 = c.points[j];
               ccp2.normalImpulse = 0.0;
               ccp2.tangentImpulse = 0.0;
            }
         }
      }
   }
   ContactSolver.prototype.SolveVelocityConstraints = function () {
      var j = 0;
      var ccp;
      var rAX = 0;
      var rAY = 0;
      var rBX = 0;
      var rBY = 0;
      var dvX = 0;
      var dvY = 0;
      var vn = 0;
      var vt = 0;
      var lambda = 0;
      var maxFriction = 0;
      var newImpulse = 0;
      var PX = 0;
      var PY = 0;
      var dX = 0;
      var dY = 0;
      var P1X = 0;
      var P1Y = 0;
      var P2X = 0;
      var P2Y = 0;
      var tMat;
      var tVec;
      for (var i = 0; i < this.constraintCount; ++i) {
         var c = this.constraints[i];
         var bodyA = c.bodyA;
         var bodyB = c.bodyB;
         var wA = bodyA.angularVelocity;
         var wB = bodyB.angularVelocity;
         var vA = bodyA.linearVelocity;
         var vB = bodyB.linearVelocity;
         var invMassA = bodyA.invMass;
         var invIA = bodyA.invI;
         var invMassB = bodyB.invMass;
         var invIB = bodyB.invI;
         var normalX = c.normal.x;
         var normalY = c.normal.y;
         var tangentX = normalY;
         var tangentY = (-normalX);
         var friction = c.friction;
         var tX = 0;
         for (j = 0;
         j < c.pointCount; j++) {
            ccp = c.points[j];
            dvX = vB.x - wB * ccp.rB.y - vA.x + wA * ccp.rA.y;
            dvY = vB.y + wB * ccp.rB.x - vA.y - wA * ccp.rA.x;
            vt = dvX * tangentX + dvY * tangentY;
            lambda = ccp.tangentMass * (-vt);
            maxFriction = friction * ccp.normalImpulse;
            newImpulse = Math.Clamp(ccp.tangentImpulse + lambda, (-maxFriction), maxFriction);
            lambda = newImpulse - ccp.tangentImpulse;
            PX = lambda * tangentX;
            PY = lambda * tangentY;
            vA.x -= invMassA * PX;
            vA.y -= invMassA * PY;
            wA -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
            vB.x += invMassB * PX;
            vB.y += invMassB * PY;
            wB += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
            ccp.tangentImpulse = newImpulse;
         }
         var tCount = parseInt(c.pointCount);
         if (c.pointCount == 1) {
            ccp = c.points[0];
            dvX = vB.x + ((-wB * ccp.rB.y)) - vA.x - ((-wA * ccp.rA.y));
            dvY = vB.y + (wB * ccp.rB.x) - vA.y - (wA * ccp.rA.x);
            vn = dvX * normalX + dvY * normalY;
            lambda = (-ccp.normalMass * (vn - ccp.velocityBias));
            newImpulse = ccp.normalImpulse + lambda;
            newImpulse = newImpulse > 0 ? newImpulse : 0.0;
            lambda = newImpulse - ccp.normalImpulse;
            PX = lambda * normalX;
            PY = lambda * normalY;
            vA.x -= invMassA * PX;
            vA.y -= invMassA * PY;
            wA -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
            vB.x += invMassB * PX;
            vB.y += invMassB * PY;
            wB += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
            ccp.normalImpulse = newImpulse;
         }
         else {
            var cp1 = c.points[0];
            var cp2 = c.points[1];
            var aX = cp1.normalImpulse;
            var aY = cp2.normalImpulse;
            var dv1X = vB.x - wB * cp1.rB.y - vA.x + wA * cp1.rA.y;
            var dv1Y = vB.y + wB * cp1.rB.x - vA.y - wA * cp1.rA.x;
            var dv2X = vB.x - wB * cp2.rB.y - vA.x + wA * cp2.rA.y;
            var dv2Y = vB.y + wB * cp2.rB.x - vA.y - wA * cp2.rA.x;
            var vn1 = dv1X * normalX + dv1Y * normalY;
            var vn2 = dv2X * normalX + dv2Y * normalY;
            var bX = vn1 - cp1.velocityBias;
            var bY = vn2 - cp2.velocityBias;
            tMat = c.K;
            bX -= tMat.col1.x * aX + tMat.col2.x * aY;
            bY -= tMat.col1.y * aX + tMat.col2.y * aY;
            var k_errorTol = 0.001;
            for (;;) {
               tMat = c.normalMass;
               var xX = (-(tMat.col1.x * bX + tMat.col2.x * bY));
               var xY = (-(tMat.col1.y * bX + tMat.col2.y * bY));
               if (xX >= 0.0 && xY >= 0.0) {
                  dX = xX - aX;
                  dY = xY - aY;
                  P1X = dX * normalX;
                  P1Y = dX * normalY;
                  P2X = dY * normalX;
                  P2Y = dY * normalY;
                  vA.x -= invMassA * (P1X + P2X);
                  vA.y -= invMassA * (P1Y + P2Y);
                  wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
                  vB.x += invMassB * (P1X + P2X);
                  vB.y += invMassB * (P1Y + P2Y);
                  wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
                  cp1.normalImpulse = xX;
                  cp2.normalImpulse = xY;
                  break;
               }
               xX = (-cp1.normalMass * bX);
               xY = 0.0;
               vn1 = 0.0;
               vn2 = c.K.col1.y * xX + bY;
               if (xX >= 0.0 && vn2 >= 0.0) {
                  dX = xX - aX;
                  dY = xY - aY;
                  P1X = dX * normalX;
                  P1Y = dX * normalY;
                  P2X = dY * normalX;
                  P2Y = dY * normalY;
                  vA.x -= invMassA * (P1X + P2X);
                  vA.y -= invMassA * (P1Y + P2Y);
                  wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
                  vB.x += invMassB * (P1X + P2X);
                  vB.y += invMassB * (P1Y + P2Y);
                  wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
                  cp1.normalImpulse = xX;
                  cp2.normalImpulse = xY;
                  break;
               }
               xX = 0.0;
               xY = (-cp2.normalMass * bY);
               vn1 = c.K.col2.x * xY + bX;
               vn2 = 0.0;
               if (xY >= 0.0 && vn1 >= 0.0) {
                  dX = xX - aX;
                  dY = xY - aY;
                  P1X = dX * normalX;
                  P1Y = dX * normalY;
                  P2X = dY * normalX;
                  P2Y = dY * normalY;
                  vA.x -= invMassA * (P1X + P2X);
                  vA.y -= invMassA * (P1Y + P2Y);
                  wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
                  vB.x += invMassB * (P1X + P2X);
                  vB.y += invMassB * (P1Y + P2Y);
                  wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
                  cp1.normalImpulse = xX;
                  cp2.normalImpulse = xY;
                  break;
               }
               xX = 0.0;
               xY = 0.0;
               vn1 = bX;
               vn2 = bY;
               if (vn1 >= 0.0 && vn2 >= 0.0) {
                  dX = xX - aX;
                  dY = xY - aY;
                  P1X = dX * normalX;
                  P1Y = dX * normalY;
                  P2X = dY * normalX;
                  P2Y = dY * normalY;
                  vA.x -= invMassA * (P1X + P2X);
                  vA.y -= invMassA * (P1Y + P2Y);
                  wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
                  vB.x += invMassB * (P1X + P2X);
                  vB.y += invMassB * (P1Y + P2Y);
                  wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
                  cp1.normalImpulse = xX;
                  cp2.normalImpulse = xY;
                  break;
               }
               break;
            }
         }
         bodyA.angularVelocity = wA;
         bodyB.angularVelocity = wB;
      }
   }
   ContactSolver.prototype.FinalizeVelocityConstraints = function () {
      for (var i = 0; i < this.constraintCount; ++i) {
         var c = this.constraints[i];
         var m = c.manifold;
         for (var j = 0; j < c.pointCount; ++j) {
            var point1 = m.points[j];
            var point2 = c.points[j];
            point1.normalImpulse = point2.normalImpulse;
            point1.tangentImpulse = point2.tangentImpulse;
         }
      }
   }
   ContactSolver.prototype.SolvePositionConstraints = function (baumgarte) {
      if (baumgarte === undefined) baumgarte = 0;
      var minSeparation = 0.0;
      for (var i = 0; i < this.constraintCount; i++) {
         var c = this.constraints[i];
         var bodyA = c.bodyA;
         var bodyB = c.bodyB;
         var invMassA = bodyA.mass * bodyA.invMass;
         var invIA = bodyA.mass * bodyA.invI;
         var invMassB = bodyB.mass * bodyB.invMass;
         var invIB = bodyB.mass * bodyB.invI;
         ContactSolver.s_psm.Initialize(c);
         var normal = ContactSolver.s_psm.normal;
         for (var j = 0; j < c.pointCount; j++) {
            var ccp = c.points[j];
            var point = ContactSolver.s_psm.points[j];
            var separation = ContactSolver.s_psm.separations[j];
            var rAX = point.x - bodyA.sweep.c.x;
            var rAY = point.y - bodyA.sweep.c.y;
            var rBX = point.x - bodyB.sweep.c.x;
            var rBY = point.y - bodyB.sweep.c.y;
            minSeparation = minSeparation < separation ? minSeparation : separation;
            var C = Math.Clamp(baumgarte * (separation + Settings._linearSlop), (-Settings._maxLinearCorrection), 0.0);
            var impulse = (-ccp.equalizedMass * C);
            var PX = impulse * normal.x;
            var PY = impulse * normal.y;bodyA.sweep.c.x -= invMassA * PX;
            bodyA.sweep.c.y -= invMassA * PY;
            bodyA.sweep.a -= invIA * (rAX * PY - rAY * PX);
            bodyA.SynchronizeTransform();
            bodyB.sweep.c.x += invMassB * PX;
            bodyB.sweep.c.y += invMassB * PY;
            bodyB.sweep.a += invIB * (rBX * PY - rBY * PX);
            bodyB.SynchronizeTransform();
         }
      }
      return minSeparation > (-1.5 * Settings._linearSlop);
   }
   Box2D.postDefs.push(function () {
      Box2D.Dynamics.Contacts.ContactSolver.s_worldManifold = new WorldManifold();
      Box2D.Dynamics.Contacts.ContactSolver.s_psm = new PositionSolverManifold();
   });
   Box2D.inherit(EdgeAndCircleContact, Box2D.Dynamics.Contacts.Contact);
   EdgeAndCircleContact.prototype.__super = Box2D.Dynamics.Contacts.Contact.prototype;
   EdgeAndCircleContact.EdgeAndCircleContact = function () {
      Box2D.Dynamics.Contacts.Contact.Contact.apply(this, arguments);
   };
   EdgeAndCircleContact.Create = function (allocator) {
      return new EdgeAndCircleContact();
   }
   EdgeAndCircleContact.Destroy = function (contact, allocator) {}
   EdgeAndCircleContact.prototype.Reset = function (fixtureA, fixtureB) {
      this.__super.Reset.call(this, fixtureA, fixtureB);
   }
   EdgeAndCircleContact.prototype.Evaluate = function () {
      var bA = this.fixtureA.GetBody();
      var bB = this.fixtureB.GetBody();
      this.CollideEdgeAndCircle(this.manifold, (this.fixtureA.GetShape() instanceof EdgeShape ? this.fixtureA.GetShape() : null), bA.xf, (this.fixtureB.GetShape() instanceof CircleShape ? this.fixtureB.GetShape() : null), bB.xf);
   }
   EdgeAndCircleContact.prototype.CollideEdgeAndCircle = function (manifold, edge, xf1, circle, xf2) {}
   Box2D.inherit(NullContact, Box2D.Dynamics.Contacts.Contact);
   NullContact.prototype.__super = Box2D.Dynamics.Contacts.Contact.prototype;
   NullContact.NullContact = function () {
      Box2D.Dynamics.Contacts.Contact.Contact.apply(this, arguments);
   };
   NullContact.prototype.NullContact = function () {
      this.__super.Contact.call(this);
   }
   NullContact.prototype.Evaluate = function () {}
   Box2D.inherit(PolyAndCircleContact, Box2D.Dynamics.Contacts.Contact);
   PolyAndCircleContact.prototype.__super = Box2D.Dynamics.Contacts.Contact.prototype;
   PolyAndCircleContact.PolyAndCircleContact = function () {
      Box2D.Dynamics.Contacts.Contact.Contact.apply(this, arguments);
   };
   PolyAndCircleContact.Create = function (allocator) {
      return new PolyAndCircleContact();
   }
   PolyAndCircleContact.Destroy = function (contact, allocator) {}
   PolyAndCircleContact.prototype.Reset = function (fixtureA, fixtureB) {
      this.__super.Reset.call(this, fixtureA, fixtureB);
      Settings.Assert(fixtureA.GetType() == Shape.e_polygonShape);
      Settings.Assert(fixtureB.GetType() == Shape.e_circleShape);
   }
   PolyAndCircleContact.prototype.Evaluate = function () {
      var bA = this.fixtureA.body;
      var bB = this.fixtureB.body;
      Collision.CollidePolygonAndCircle(this.manifold, (this.fixtureA.GetShape() instanceof PolygonShape ? this.fixtureA.GetShape() : null), bA.xf, (this.fixtureB.GetShape() instanceof CircleShape ? this.fixtureB.GetShape() : null), bB.xf);
   }
   Box2D.inherit(PolyAndEdgeContact, Box2D.Dynamics.Contacts.Contact);
   PolyAndEdgeContact.prototype.__super = Box2D.Dynamics.Contacts.Contact.prototype;
   PolyAndEdgeContact.PolyAndEdgeContact = function () {
      Box2D.Dynamics.Contacts.Contact.Contact.apply(this, arguments);
   };
   PolyAndEdgeContact.Create = function (allocator) {
      return new PolyAndEdgeContact();
   }
   PolyAndEdgeContact.Destroy = function (contact, allocator) {}
   PolyAndEdgeContact.prototype.Reset = function (fixtureA, fixtureB) {
      this.__super.Reset.call(this, fixtureA, fixtureB);
      Settings.Assert(fixtureA.GetType() == Shape.e_polygonShape);
      Settings.Assert(fixtureB.GetType() == Shape.e_edgeShape);
   }
   PolyAndEdgeContact.prototype.Evaluate = function () {
      var bA = this.fixtureA.GetBody();
      var bB = this.fixtureB.GetBody();
      this.CollidePolyAndEdge(this.manifold, (this.fixtureA.GetShape() instanceof PolygonShape ? this.fixtureA.GetShape() : null), bA.xf, (this.fixtureB.GetShape() instanceof EdgeShape ? this.fixtureB.GetShape() : null), bB.xf);
   }
   PolyAndEdgeContact.prototype.CollidePolyAndEdge = function (manifold, polygon, xf1, edge, xf2) {}
   Box2D.inherit(PolygonContact, Box2D.Dynamics.Contacts.Contact);
   PolygonContact.prototype.__super = Box2D.Dynamics.Contacts.Contact.prototype;
   PolygonContact.PolygonContact = function () {
      Box2D.Dynamics.Contacts.Contact.Contact.apply(this, arguments);
   };
   PolygonContact.Create = function (allocator) {
      return new PolygonContact();
   }
   PolygonContact.Destroy = function (contact, allocator) {}
   PolygonContact.prototype.Reset = function (fixtureA, fixtureB) {
      this.__super.Reset.call(this, fixtureA, fixtureB);
   }
   PolygonContact.prototype.Evaluate = function () {
      var bA = this.fixtureA.GetBody();
      var bB = this.fixtureB.GetBody();
      Collision.CollidePolygons(this.manifold, (this.fixtureA.GetShape() instanceof PolygonShape ? this.fixtureA.GetShape() : null), bA.xf, (this.fixtureB.GetShape() instanceof PolygonShape ? this.fixtureB.GetShape() : null), bB.xf);
   }
   PositionSolverManifold.PositionSolverManifold = function () {};
   PositionSolverManifold.prototype.PositionSolverManifold = function () {
      this.normal = new Vec2();
      this.separations = new Vector_a2j_Number(Settings._maxManifoldPoints);
      this.points = new Vector(Settings._maxManifoldPoints);
      for (var i = 0; i < Settings._maxManifoldPoints; i++) {
         this.points[i] = new Vec2();
      }
   }
   PositionSolverManifold.prototype.Initialize = function (cc) {
      Settings.Assert(cc.pointCount > 0);
      var i = 0;
      var clipPointX = 0;
      var clipPointY = 0;
      var tMat;
      var tVec;
      var planePointX = 0;
      var planePointY = 0;
      switch (cc.type) {
      case Manifold.e_circles:
         {
            tMat = cc.bodyA.xf.R;
            tVec = cc.localPoint;
            var pointAX = cc.bodyA.xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
            var pointAY = cc.bodyA.xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
            tMat = cc.bodyB.xf.R;
            tVec = cc.points[0].localPoint;
            var pointBX = cc.bodyB.xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
            var pointBY = cc.bodyB.xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
            var dX = pointBX - pointAX;
            var dY = pointBY - pointAY;
            var d2 = dX * dX + dY * dY;
            if (d2 > Number.MIN_VALUE * Number.MIN_VALUE) {
               var d = Math.sqrt(d2);
               this.normal.x = dX / d;
               this.normal.y = dY / d;
            }
            else {
               this.normal.x = 1.0;
               this.normal.y = 0.0;
            }
            this.points[0].x = 0.5 * (pointAX + pointBX);
            this.points[0].y = 0.5 * (pointAY + pointBY);
            this.separations[0] = dX * this.normal.x + dY * this.normal.y - cc.radius;
         }
         break;
      case Manifold.e_faceA:
         {
            tMat = cc.bodyA.xf.R;
            tVec = cc.localPlaneNormal;
            this.normal.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            this.normal.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            tMat = cc.bodyA.xf.R;
            tVec = cc.localPoint;
            planePointX = cc.bodyA.xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
            planePointY = cc.bodyA.xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
            tMat = cc.bodyB.xf.R;
            for (i = 0;
            i < cc.pointCount; ++i) {
               tVec = cc.points[i].localPoint;
               clipPointX = cc.bodyB.xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
               clipPointY = cc.bodyB.xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
               this.separations[i] = (clipPointX - planePointX) * this.normal.x + (clipPointY - planePointY) * this.normal.y - cc.radius;
               this.points[i].x = clipPointX;
               this.points[i].y = clipPointY;
            }
         }
         break;
      case Manifold.e_faceB:
         {
            tMat = cc.bodyB.xf.R;
            tVec = cc.localPlaneNormal;
            this.normal.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            this.normal.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            tMat = cc.bodyB.xf.R;
            tVec = cc.localPoint;
            planePointX = cc.bodyB.xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
            planePointY = cc.bodyB.xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
            tMat = cc.bodyA.xf.R;
            for (i = 0;
            i < cc.pointCount; ++i) {
               tVec = cc.points[i].localPoint;
               clipPointX = cc.bodyA.xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
               clipPointY = cc.bodyA.xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
               this.separations[i] = (clipPointX - planePointX) * this.normal.x + (clipPointY - planePointY) * this.normal.y - cc.radius;
               this.points[i].Set(clipPointX, clipPointY);
            }
            this.normal.x *= (-1);
            this.normal.y *= (-1);
         }
         break;
      }
   }
   Box2D.postDefs.push(function () {
      Box2D.Dynamics.Contacts.PositionSolverManifold.circlePointA = new Vec2();
      Box2D.Dynamics.Contacts.PositionSolverManifold.circlePointB = new Vec2();
   });
})();
(function () {
   var Body = Box2D.Dynamics.Body,
      BodyDef = Box2D.Dynamics.BodyDef,
      ContactFilter = Box2D.Dynamics.ContactFilter,
      ContactImpulse = Box2D.Dynamics.ContactImpulse,
      ContactListener = Box2D.Dynamics.ContactListener,
      ContactManager = Box2D.Dynamics.ContactManager,
      DebugDraw = Box2D.Dynamics.DebugDraw,
      DestructionListener = Box2D.Dynamics.DestructionListener,
      FilterData = Box2D.Dynamics.FilterData,
      Fixture = Box2D.Dynamics.Fixture,
      FixtureDef = Box2D.Dynamics.FixtureDef,
      Island = Box2D.Dynamics.Island,
      TimeStep = Box2D.Dynamics.TimeStep,
      World = Box2D.Dynamics.World,
      Mat22 = Box2D.Common.Math.Mat22,
      Mat33 = Box2D.Common.Math.Mat33,
      Math = Box2D.Common.Math.Math,
      Sweep = Box2D.Common.Math.Sweep,
      Transform = Box2D.Common.Math.Transform,
      Vec2 = Box2D.Common.Math.Vec2,
      Vec3 = Box2D.Common.Math.Vec3,
      Color = Box2D.Common.Color,
      internal = Box2D.Common.internal,
      Settings = Box2D.Common.Settings,
      CircleShape = Box2D.Collision.Shapes.CircleShape,
      EdgeChainDef = Box2D.Collision.Shapes.EdgeChainDef,
      EdgeShape = Box2D.Collision.Shapes.EdgeShape,
      MassData = Box2D.Collision.Shapes.MassData,
      PolygonShape = Box2D.Collision.Shapes.PolygonShape,
      Shape = Box2D.Collision.Shapes.Shape,
      BuoyancyController = Box2D.Dynamics.Controllers.BuoyancyController,
      ConstantAccelController = Box2D.Dynamics.Controllers.ConstantAccelController,
      ConstantForceController = Box2D.Dynamics.Controllers.ConstantForceController,
      Controller = Box2D.Dynamics.Controllers.Controller,
      ControllerEdge = Box2D.Dynamics.Controllers.ControllerEdge,
      GravityController = Box2D.Dynamics.Controllers.GravityController,
      TensorDampingController = Box2D.Dynamics.Controllers.TensorDampingController;

   Box2D.inherit(BuoyancyController, Box2D.Dynamics.Controllers.Controller);
   BuoyancyController.prototype.__super = Box2D.Dynamics.Controllers.Controller.prototype;
   BuoyancyController.BuoyancyController = function () {
      Box2D.Dynamics.Controllers.Controller.Controller.apply(this, arguments);
      this.normal = new Vec2(0, (-1));
      this.offset = 0;
      this.density = 0;
      this.velocity = new Vec2(0, 0);
      this.linearDrag = 2;
      this.angularDrag = 1;
      this.useDensity = false;
      this.useWorldGravity = true;
      this.gravity = null;
   };
   BuoyancyController.prototype.Step = function (step) {
      if (!this.bodyList) return;
      if (this.useWorldGravity) {
         this.gravity = this.GetWorld().GetGravity().Copy();
      }
      for (var i = this.bodyList; i; i = i.nextBody) {
         var body = i.body;
         if (body.IsAwake() == false) {
            continue;
         }
         var areac = new Vec2();
         var massc = new Vec2();
         var area = 0.0;
         var mass = 0.0;
         for (var fixture = body.GetFixtureList(); fixture; fixture = fixture.GetNext()) {
            var sc = new Vec2();
            var sarea = fixture.GetShape().ComputeSubmergedArea(this.normal, this.offset, body.GetTransform(), sc);
            area += sarea;
            areac.x += sarea * sc.x;
            areac.y += sarea * sc.y;
            var shapeDensity = 0;
            if (this.useDensity) {
               shapeDensity = 1;
            }
            else {
               shapeDensity = 1;
            }
            mass += sarea * shapeDensity;
            massc.x += sarea * sc.x * shapeDensity;
            massc.y += sarea * sc.y * shapeDensity;
         }
         areac.x /= area;
         areac.y /= area;
         massc.x /= mass;
         massc.y /= mass;
         if (area < Number.MIN_VALUE) continue;
         var buoyancyForce = this.gravity.GetNegative();
         buoyancyForce.Multiply(this.density * area);
         body.ApplyForce(buoyancyForce, massc);
         var dragForce = body.GetLinearVelocityFromWorldPoint(areac);
         dragForce.Subtract(this.velocity);
         dragForce.Multiply((-this.linearDrag * area));
         body.ApplyForce(dragForce, areac);
         body.ApplyTorque((-body.GetInertia() / body.GetMass() * area * body.GetAngularVelocity() * this.angularDrag));
      }
   }
   BuoyancyController.prototype.Draw = function (debugDraw) {
      var r = 1000;
      var p1 = new Vec2();
      var p2 = new Vec2();
      p1.x = this.normal.x * this.offset + this.normal.y * r;
      p1.y = this.normal.y * this.offset - this.normal.x * r;
      p2.x = this.normal.x * this.offset - this.normal.y * r;
      p2.y = this.normal.y * this.offset + this.normal.x * r;
      var color = new Color(0, 0, 1);
      debugDraw.DrawSegment(p1, p2, color);
   }
   Box2D.inherit(ConstantAccelController, Box2D.Dynamics.Controllers.Controller);
   ConstantAccelController.prototype.__super = Box2D.Dynamics.Controllers.Controller.prototype;
   ConstantAccelController.ConstantAccelController = function () {
      Box2D.Dynamics.Controllers.Controller.Controller.apply(this, arguments);
      this.A = new Vec2(0, 0);
   };
   ConstantAccelController.prototype.Step = function (step) {
      var smallA = new Vec2(this.A.x * step.dt, this.A.y * step.dt);
      for (var i = this.bodyList; i; i = i.nextBody) {
         var body = i.body;
         if (!body.IsAwake()) continue;
         body.SetLinearVelocity(new Vec2(body.GetLinearVelocity().x + smallA.x, body.GetLinearVelocity().y + smallA.y));
      }
   }
   Box2D.inherit(ConstantForceController, Box2D.Dynamics.Controllers.Controller);
   ConstantForceController.prototype.__super = Box2D.Dynamics.Controllers.Controller.prototype;
   ConstantForceController.ConstantForceController = function () {
      Box2D.Dynamics.Controllers.Controller.Controller.apply(this, arguments);
      this.F = new Vec2(0, 0);
   };
   ConstantForceController.prototype.Step = function (step) {
      for (var i = this.bodyList; i; i = i.nextBody) {
         var body = i.body;
         if (!body.IsAwake()) continue;
         body.ApplyForce(this.F, body.GetWorldCenter());
      }
   }
   Controller.Controller = function () {};
   Controller.prototype.Step = function (step) {}
   Controller.prototype.Draw = function (debugDraw) {}
   Controller.prototype.AddBody = function (body) {
      var edge = new ControllerEdge();
      edge.controller = this;
      edge.body = body;
      edge.nextBody = this.bodyList;
      edge.prevBody = null;
      this.bodyList = edge;
      if (edge.nextBody) edge.nextBody.prevBody = edge;
      this.bodyCount++;
      edge.nextController = body.controllerList;
      edge.prevController = null;
      body.controllerList = edge;
      if (edge.nextController) edge.nextController.prevController = edge;
      body.controllerCount++;
   }
   Controller.prototype.RemoveBody = function (body) {
      var edge = body.controllerList;
      while (edge && edge.controller != this)
      edge = edge.nextController;
      if (edge.prevBody) edge.prevBody.nextBody = edge.nextBody;
      if (edge.nextBody) edge.nextBody.prevBody = edge.prevBody;
      if (edge.nextController) edge.nextController.prevController = edge.prevController;
      if (edge.prevController) edge.prevController.nextController = edge.nextController;
      if (this.bodyList == edge) this.bodyList = edge.nextBody;
      if (body.controllerList == edge) body.controllerList = edge.nextController;
      body.controllerCount--;
      this.bodyCount--;
   }
   Controller.prototype.Clear = function () {
      while (this.bodyList)
      this.RemoveBody(this.bodyList.body);
   }
   Controller.prototype.GetNext = function () {
      return this.next;
   }
   Controller.prototype.GetWorld = function () {
      return this.world;
   }
   Controller.prototype.GetBodyList = function () {
      return this.bodyList;
   }
   ControllerEdge.ControllerEdge = function () {};
   Box2D.inherit(GravityController, Box2D.Dynamics.Controllers.Controller);
   GravityController.prototype.__super = Box2D.Dynamics.Controllers.Controller.prototype;
   GravityController.GravityController = function () {
      Box2D.Dynamics.Controllers.Controller.Controller.apply(this, arguments);
      this.G = 1;
      this.invSqr = true;
   };
   GravityController.prototype.Step = function (step) {
      var i = null;
      var body1 = null;
      var p1 = null;
      var mass1 = 0;
      var j = null;
      var body2 = null;
      var p2 = null;
      var dx = 0;
      var dy = 0;
      var r2 = 0;
      var f = null;
      if (this.invSqr) {
         for (i = this.bodyList;
         i; i = i.nextBody) {
            body1 = i.body;
            p1 = body1.GetWorldCenter();
            mass1 = body1.GetMass();
            for (j = this.bodyList;
            j != i; j = j.nextBody) {
               body2 = j.body;
               p2 = body2.GetWorldCenter();
               dx = p2.x - p1.x;
               dy = p2.y - p1.y;
               r2 = dx * dx + dy * dy;
               if (r2 < Number.MIN_VALUE) continue;
               f = new Vec2(dx, dy);
               f.Multiply(this.G / r2 / Math.sqrt(r2) * mass1 * body2.GetMass());
               if (body1.IsAwake()) body1.ApplyForce(f, p1);
               f.Multiply((-1));
               if (body2.IsAwake()) body2.ApplyForce(f, p2);
            }
         }
      }
      else {
         for (i = this.bodyList;
         i; i = i.nextBody) {
            body1 = i.body;
            p1 = body1.GetWorldCenter();
            mass1 = body1.GetMass();
            for (j = this.bodyList;
            j != i; j = j.nextBody) {
               body2 = j.body;
               p2 = body2.GetWorldCenter();
               dx = p2.x - p1.x;
               dy = p2.y - p1.y;
               r2 = dx * dx + dy * dy;
               if (r2 < Number.MIN_VALUE) continue;
               f = new Vec2(dx, dy);
               f.Multiply(this.G / r2 * mass1 * body2.GetMass());
               if (body1.IsAwake()) body1.ApplyForce(f, p1);
               f.Multiply((-1));
               if (body2.IsAwake()) body2.ApplyForce(f, p2);
            }
         }
      }
   }
   Box2D.inherit(TensorDampingController, Box2D.Dynamics.Controllers.Controller);
   TensorDampingController.prototype.__super = Box2D.Dynamics.Controllers.Controller.prototype;
   TensorDampingController.TensorDampingController = function () {
      Box2D.Dynamics.Controllers.Controller.Controller.apply(this, arguments);
      this.T = new Mat22();
      this.maxTimestep = 0;
   };
   TensorDampingController.prototype.SetAxisAligned = function (xDamping, yDamping) {
      if (xDamping === undefined) xDamping = 0;
      if (yDamping === undefined) yDamping = 0;
      this.T.col1.x = (-xDamping);
      this.T.col1.y = 0;
      this.T.col2.x = 0;
      this.T.col2.y = (-yDamping);
      if (xDamping > 0 || yDamping > 0) {
         this.maxTimestep = 1 / Math.max(xDamping, yDamping);
      }
      else {
         this.maxTimestep = 0;
      }
   }
   TensorDampingController.prototype.Step = function (step) {
      var timestep = step.dt;
      if (timestep <= Number.MIN_VALUE) return;
      if (timestep > this.maxTimestep && this.maxTimestep > 0) timestep = this.maxTimestep;
      for (var i = this.bodyList; i; i = i.nextBody) {
         var body = i.body;
         if (!body.IsAwake()) {
            continue;
         }
         var damping = body.GetWorldVector(Math.MulMV(this.T, body.GetLocalVector(body.GetLinearVelocity())));
         body.SetLinearVelocity(new Vec2(body.GetLinearVelocity().x + damping.x * timestep, body.GetLinearVelocity().y + damping.y * timestep));
      }
   }
})();
(function () {
   var Color = Box2D.Common.Color,
      internal = Box2D.Common.internal,
      Settings = Box2D.Common.Settings,
      Mat22 = Box2D.Common.Math.Mat22,
      Mat33 = Box2D.Common.Math.Mat33,
      Math = Box2D.Common.Math.Math,
      Sweep = Box2D.Common.Math.Sweep,
      Transform = Box2D.Common.Math.Transform,
      Vec2 = Box2D.Common.Math.Vec2,
      Vec3 = Box2D.Common.Math.Vec3,
      DistanceJoint = Box2D.Dynamics.Joints.DistanceJoint,
      DistanceJointDef = Box2D.Dynamics.Joints.DistanceJointDef,
      FrictionJoint = Box2D.Dynamics.Joints.FrictionJoint,
      FrictionJointDef = Box2D.Dynamics.Joints.FrictionJointDef,
      GearJoint = Box2D.Dynamics.Joints.GearJoint,
      GearJointDef = Box2D.Dynamics.Joints.GearJointDef,
      Jacobian = Box2D.Dynamics.Joints.Jacobian,
      Joint = Box2D.Dynamics.Joints.Joint,
      JointDef = Box2D.Dynamics.Joints.JointDef,
      JointEdge = Box2D.Dynamics.Joints.JointEdge,
      LineJoint = Box2D.Dynamics.Joints.LineJoint,
      LineJointDef = Box2D.Dynamics.Joints.LineJointDef,
      MouseJoint = Box2D.Dynamics.Joints.MouseJoint,
      MouseJointDef = Box2D.Dynamics.Joints.MouseJointDef,
      PrismaticJoint = Box2D.Dynamics.Joints.PrismaticJoint,
      PrismaticJointDef = Box2D.Dynamics.Joints.PrismaticJointDef,
      PulleyJoint = Box2D.Dynamics.Joints.PulleyJoint,
      PulleyJointDef = Box2D.Dynamics.Joints.PulleyJointDef,
      RevoluteJoint = Box2D.Dynamics.Joints.RevoluteJoint,
      RevoluteJointDef = Box2D.Dynamics.Joints.RevoluteJointDef,
      WeldJoint = Box2D.Dynamics.Joints.WeldJoint,
      WeldJointDef = Box2D.Dynamics.Joints.WeldJointDef,
      Body = Box2D.Dynamics.Body,
      BodyDef = Box2D.Dynamics.BodyDef,
      ContactFilter = Box2D.Dynamics.ContactFilter,
      ContactImpulse = Box2D.Dynamics.ContactImpulse,
      ContactListener = Box2D.Dynamics.ContactListener,
      ContactManager = Box2D.Dynamics.ContactManager,
      DebugDraw = Box2D.Dynamics.DebugDraw,
      DestructionListener = Box2D.Dynamics.DestructionListener,
      FilterData = Box2D.Dynamics.FilterData,
      Fixture = Box2D.Dynamics.Fixture,
      FixtureDef = Box2D.Dynamics.FixtureDef,
      Island = Box2D.Dynamics.Island,
      TimeStep = Box2D.Dynamics.TimeStep,
      World = Box2D.Dynamics.World;

   Box2D.inherit(DistanceJoint, Box2D.Dynamics.Joints.Joint);
   DistanceJoint.prototype.__super = Box2D.Dynamics.Joints.Joint.prototype;
   DistanceJoint.DistanceJoint = function () {
      Box2D.Dynamics.Joints.Joint.Joint.apply(this, arguments);
      this.localAnchor1 = new Vec2();
      this.localAnchor2 = new Vec2();
      this.u = new Vec2();
   };
   DistanceJoint.prototype.GetAnchorA = function () {
      return this.bodyA.GetWorldPoint(this.localAnchor1);
   }
   DistanceJoint.prototype.GetAnchorB = function () {
      return this.bodyB.GetWorldPoint(this.localAnchor2);
   }
   DistanceJoint.prototype.GetReactionForce = function (inv_dt) {
      if (inv_dt === undefined) inv_dt = 0;
      return new Vec2(inv_dt * this.impulse * this.u.x, inv_dt * this.impulse * this.u.y);
   }
   DistanceJoint.prototype.GetReactionTorque = function (inv_dt) {
      if (inv_dt === undefined) inv_dt = 0;
      return 0.0;
   }
   DistanceJoint.prototype.GetLength = function () {
      return this.length;
   }
   DistanceJoint.prototype.SetLength = function (length) {
      if (length === undefined) length = 0;
      this.length = length;
   }
   DistanceJoint.prototype.GetFrequency = function () {
      return this.frequencyHz;
   }
   DistanceJoint.prototype.SetFrequency = function (hz) {
      if (hz === undefined) hz = 0;
      this.frequencyHz = hz;
   }
   DistanceJoint.prototype.GetDampingRatio = function () {
      return this.dampingRatio;
   }
   DistanceJoint.prototype.SetDampingRatio = function (ratio) {
      if (ratio === undefined) ratio = 0;
      this.dampingRatio = ratio;
   }
   DistanceJoint.prototype.DistanceJoint = function (def) {
      this.__super.Joint.call(this, def);
      var tMat;
      var tX = 0;
      var tY = 0;
      this.localAnchor1.SetV(def.localAnchorA);
      this.localAnchor2.SetV(def.localAnchorB);
      this.length = def.length;
      this.frequencyHz = def.frequencyHz;
      this.dampingRatio = def.dampingRatio;
      this.impulse = 0.0;
      this.gamma = 0.0;
      this.bias = 0.0;
   }
   DistanceJoint.prototype.InitVelocityConstraints = function (step) {
      var tMat;
      var tX = 0;
      var bA = this.bodyA;
      var bB = this.bodyB;
      tMat = bA.xf.R;
      var r1X = this.localAnchor1.x - bA.sweep.localCenter.x;
      var r1Y = this.localAnchor1.y - bA.sweep.localCenter.y;
      tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
      r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
      r1X = tX;
      tMat = bB.xf.R;
      var r2X = this.localAnchor2.x - bB.sweep.localCenter.x;
      var r2Y = this.localAnchor2.y - bB.sweep.localCenter.y;
      tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
      r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
      r2X = tX;
      this.u.x = bB.sweep.c.x + r2X - bA.sweep.c.x - r1X;
      this.u.y = bB.sweep.c.y + r2Y - bA.sweep.c.y - r1Y;
      var length = Math.sqrt(this.u.x * this.u.x + this.u.y * this.u.y);
      if (length > Settings._linearSlop) {
         this.u.Multiply(1.0 / length);
      }
      else {
         this.u.SetZero();
      }
      var cr1u = (r1X * this.u.y - r1Y * this.u.x);
      var cr2u = (r2X * this.u.y - r2Y * this.u.x);
      var invMass = bA.invMass + bA.invI * cr1u * cr1u + bB.invMass + bB.invI * cr2u * cr2u;
      this.mass = invMass != 0.0 ? 1.0 / invMass : 0.0;
      if (this.frequencyHz > 0.0) {
         var C = length - this.length;
         var omega = 2.0 * Math.PI * this.frequencyHz;
         var d = 2.0 * this.mass * this.dampingRatio * omega;
         var k = this.mass * omega * omega;
         this.gamma = step.dt * (d + step.dt * k);
         this.gamma = this.gamma != 0.0 ? 1 / this.gamma : 0.0;
         this.bias = C * step.dt * k * this.gamma;
         this.mass = invMass + this.gamma;
         this.mass = this.mass != 0.0 ? 1.0 / this.mass : 0.0;
      }
      if (step.warmStarting) {
         this.impulse *= step.dtRatio;
         var PX = this.impulse * this.u.x;
         var PY = this.impulse * this.u.y;
         bA.linearVelocity.x -= bA.invMass * PX;
         bA.linearVelocity.y -= bA.invMass * PY;
         bA.angularVelocity -= bA.invI * (r1X * PY - r1Y * PX);
         bB.linearVelocity.x += bB.invMass * PX;
         bB.linearVelocity.y += bB.invMass * PY;
         bB.angularVelocity += bB.invI * (r2X * PY - r2Y * PX);
      }
      else {
         this.impulse = 0.0;
      }
   }
   DistanceJoint.prototype.SolveVelocityConstraints = function (step) {
      var tMat;
      var bA = this.bodyA;
      var bB = this.bodyB;
      tMat = bA.xf.R;
      var r1X = this.localAnchor1.x - bA.sweep.localCenter.x;
      var r1Y = this.localAnchor1.y - bA.sweep.localCenter.y;
      var tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
      r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
      r1X = tX;
      tMat = bB.xf.R;
      var r2X = this.localAnchor2.x - bB.sweep.localCenter.x;
      var r2Y = this.localAnchor2.y - bB.sweep.localCenter.y;
      tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
      r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
      r2X = tX;
      var v1X = bA.linearVelocity.x + ((-bA.angularVelocity * r1Y));
      var v1Y = bA.linearVelocity.y + (bA.angularVelocity * r1X);
      var v2X = bB.linearVelocity.x + ((-bB.angularVelocity * r2Y));
      var v2Y = bB.linearVelocity.y + (bB.angularVelocity * r2X);
      var Cdot = (this.u.x * (v2X - v1X) + this.u.y * (v2Y - v1Y));
      var impulse = (-this.mass * (Cdot + this.bias + this.gamma * this.impulse));
      this.impulse += impulse;
      var PX = impulse * this.u.x;
      var PY = impulse * this.u.y;
      bA.linearVelocity.x -= bA.invMass * PX;
      bA.linearVelocity.y -= bA.invMass * PY;
      bA.angularVelocity -= bA.invI * (r1X * PY - r1Y * PX);
      bB.linearVelocity.x += bB.invMass * PX;
      bB.linearVelocity.y += bB.invMass * PY;
      bB.angularVelocity += bB.invI * (r2X * PY - r2Y * PX);
   }
   DistanceJoint.prototype.SolvePositionConstraints = function (baumgarte) {
      if (baumgarte === undefined) baumgarte = 0;
      var tMat;
      if (this.frequencyHz > 0.0) {
         return true;
      }
      var bA = this.bodyA;
      var bB = this.bodyB;
      tMat = bA.xf.R;
      var r1X = this.localAnchor1.x - bA.sweep.localCenter.x;
      var r1Y = this.localAnchor1.y - bA.sweep.localCenter.y;
      var tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
      r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
      r1X = tX;
      tMat = bB.xf.R;
      var r2X = this.localAnchor2.x - bB.sweep.localCenter.x;
      var r2Y = this.localAnchor2.y - bB.sweep.localCenter.y;
      tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
      r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
      r2X = tX;
      var dX = bB.sweep.c.x + r2X - bA.sweep.c.x - r1X;
      var dY = bB.sweep.c.y + r2Y - bA.sweep.c.y - r1Y;
      var length = Math.sqrt(dX * dX + dY * dY);
      dX /= length;
      dY /= length;
      var C = length - this.length;
      C = Math.Clamp(C, (-Settings._maxLinearCorrection), Settings._maxLinearCorrection);
      var impulse = (-this.mass * C);
      this.u.Set(dX, dY);
      var PX = impulse * this.u.x;
      var PY = impulse * this.u.y;
      bA.sweep.c.x -= bA.invMass * PX;
      bA.sweep.c.y -= bA.invMass * PY;
      bA.sweep.a -= bA.invI * (r1X * PY - r1Y * PX);
      bB.sweep.c.x += bB.invMass * PX;
      bB.sweep.c.y += bB.invMass * PY;
      bB.sweep.a += bB.invI * (r2X * PY - r2Y * PX);
      bA.SynchronizeTransform();
      bB.SynchronizeTransform();
      return Math.Abs(C) < Settings._linearSlop;
   }
   Box2D.inherit(DistanceJointDef, Box2D.Dynamics.Joints.JointDef);
   DistanceJointDef.prototype.__super = Box2D.Dynamics.Joints.JointDef.prototype;
   DistanceJointDef.DistanceJointDef = function () {
      Box2D.Dynamics.Joints.JointDef.JointDef.apply(this, arguments);
      this.localAnchorA = new Vec2();
      this.localAnchorB = new Vec2();
   };
   DistanceJointDef.prototype.DistanceJointDef = function () {
      this.__super.JointDef.call(this);
      this.type = Joint.e_distanceJoint;
      this.length = 1.0;
      this.frequencyHz = 0.0;
      this.dampingRatio = 0.0;
   }
   DistanceJointDef.prototype.Initialize = function (bA, bB, anchorA, anchorB) {
      this.bodyA = bA;
      this.bodyB = bB;
      this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchorA));
      this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchorB));
      var dX = anchorB.x - anchorA.x;
      var dY = anchorB.y - anchorA.y;
      this.length = Math.sqrt(dX * dX + dY * dY);
      this.frequencyHz = 0.0;
      this.dampingRatio = 0.0;
   }
   Box2D.inherit(FrictionJoint, Box2D.Dynamics.Joints.Joint);
   FrictionJoint.prototype.__super = Box2D.Dynamics.Joints.Joint.prototype;
   FrictionJoint.FrictionJoint = function () {
      Box2D.Dynamics.Joints.Joint.Joint.apply(this, arguments);
      this.localAnchorA = new Vec2();
      this.localAnchorB = new Vec2();
      this.linearMass = new Mat22();
      this.linearImpulse = new Vec2();
   };
   FrictionJoint.prototype.GetAnchorA = function () {
      return this.bodyA.GetWorldPoint(this.localAnchorA);
   }
   FrictionJoint.prototype.GetAnchorB = function () {
      return this.bodyB.GetWorldPoint(this.localAnchorB);
   }
   FrictionJoint.prototype.GetReactionForce = function (inv_dt) {
      if (inv_dt === undefined) inv_dt = 0;
      return new Vec2(inv_dt * this.linearImpulse.x, inv_dt * this.linearImpulse.y);
   }
   FrictionJoint.prototype.GetReactionTorque = function (inv_dt) {
      if (inv_dt === undefined) inv_dt = 0;
      return inv_dt * this.angularImpulse;
   }
   FrictionJoint.prototype.SetMaxForce = function (force) {
      if (force === undefined) force = 0;
      this.maxForce = force;
   }
   FrictionJoint.prototype.GetMaxForce = function () {
      return this.maxForce;
   }
   FrictionJoint.prototype.SetMaxTorque = function (torque) {
      if (torque === undefined) torque = 0;
      this.maxTorque = torque;
   }
   FrictionJoint.prototype.GetMaxTorque = function () {
      return this.maxTorque;
   }
   FrictionJoint.prototype.FrictionJoint = function (def) {
      this.__super.Joint.call(this, def);
      this.localAnchorA.SetV(def.localAnchorA);
      this.localAnchorB.SetV(def.localAnchorB);
      this.linearMass.SetZero();
      this.angularMass = 0.0;
      this.linearImpulse.SetZero();
      this.angularImpulse = 0.0;
      this.maxForce = def.maxForce;
      this.maxTorque = def.maxTorque;
   }
   FrictionJoint.prototype.InitVelocityConstraints = function (step) {
      var tMat;
      var tX = 0;
      var bA = this.bodyA;
      var bB = this.bodyB;
      tMat = bA.xf.R;
      var rAX = this.localAnchorA.x - bA.sweep.localCenter.x;
      var rAY = this.localAnchorA.y - bA.sweep.localCenter.y;
      tX = (tMat.col1.x * rAX + tMat.col2.x * rAY);
      rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY);
      rAX = tX;
      tMat = bB.xf.R;
      var rBX = this.localAnchorB.x - bB.sweep.localCenter.x;
      var rBY = this.localAnchorB.y - bB.sweep.localCenter.y;
      tX = (tMat.col1.x * rBX + tMat.col2.x * rBY);
      rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY);
      rBX = tX;
      var mA = bA.invMass;
      var mB = bB.invMass;
      var iA = bA.invI;
      var iB = bB.invI;
      var K = new Mat22();
      K.col1.x = mA + mB;
      K.col2.x = 0.0;
      K.col1.y = 0.0;
      K.col2.y = mA + mB;
      K.col1.x += iA * rAY * rAY;
      K.col2.x += (-iA * rAX * rAY);
      K.col1.y += (-iA * rAX * rAY);
      K.col2.y += iA * rAX * rAX;
      K.col1.x += iB * rBY * rBY;
      K.col2.x += (-iB * rBX * rBY);
      K.col1.y += (-iB * rBX * rBY);
      K.col2.y += iB * rBX * rBX;
      K.GetInverse(this.linearMass);
      this.angularMass = iA + iB;
      if (this.angularMass > 0.0) {
         this.angularMass = 1.0 / this.angularMass;
      }
      if (step.warmStarting) {
         this.linearImpulse.x *= step.dtRatio;
         this.linearImpulse.y *= step.dtRatio;
         this.angularImpulse *= step.dtRatio;
         var P = this.linearImpulse;
         bA.linearVelocity.x -= mA * P.x;
         bA.linearVelocity.y -= mA * P.y;
         bA.angularVelocity -= iA * (rAX * P.y - rAY * P.x + this.angularImpulse);
         bB.linearVelocity.x += mB * P.x;
         bB.linearVelocity.y += mB * P.y;
         bB.angularVelocity += iB * (rBX * P.y - rBY * P.x + this.angularImpulse);
      }
      else {
         this.linearImpulse.SetZero();
         this.angularImpulse = 0.0;
      }
   }
   FrictionJoint.prototype.SolveVelocityConstraints = function (step) {
      var tMat;
      var tX = 0;
      var bA = this.bodyA;
      var bB = this.bodyB;
      var vA = bA.linearVelocity;
      var wA = bA.angularVelocity;
      var vB = bB.linearVelocity;
      var wB = bB.angularVelocity;
      var mA = bA.invMass;
      var mB = bB.invMass;
      var iA = bA.invI;
      var iB = bB.invI;
      tMat = bA.xf.R;
      var rAX = this.localAnchorA.x - bA.sweep.localCenter.x;
      var rAY = this.localAnchorA.y - bA.sweep.localCenter.y;
      tX = (tMat.col1.x * rAX + tMat.col2.x * rAY);
      rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY);
      rAX = tX;
      tMat = bB.xf.R;
      var rBX = this.localAnchorB.x - bB.sweep.localCenter.x;
      var rBY = this.localAnchorB.y - bB.sweep.localCenter.y;
      tX = (tMat.col1.x * rBX + tMat.col2.x * rBY);
      rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY);
      rBX = tX;
      var maxImpulse = 0; {
         var Cdot = wB - wA;
         var impulse = (-this.angularMass * Cdot);
         var oldImpulse = this.angularImpulse;
         maxImpulse = step.dt * this.maxTorque;
         this.angularImpulse = Math.Clamp(this.angularImpulse + impulse, (-maxImpulse), maxImpulse);
         impulse = this.angularImpulse - oldImpulse;
         wA -= iA * impulse;
         wB += iB * impulse;
      } {
         var CdotX = vB.x - wB * rBY - vA.x + wA * rAY;
         var CdotY = vB.y + wB * rBX - vA.y - wA * rAX;
         var impulseV = Math.MulMV(this.linearMass, new Vec2((-CdotX), (-CdotY)));
         var oldImpulseV = this.linearImpulse.Copy();
         this.linearImpulse.Add(impulseV);
         maxImpulse = step.dt * this.maxForce;
         if (this.linearImpulse.LengthSquared() > maxImpulse * maxImpulse) {
            this.linearImpulse.Normalize();
            this.linearImpulse.Multiply(maxImpulse);
         }
         impulseV = Math.SubtractVV(this.linearImpulse, oldImpulseV);
         vA.x -= mA * impulseV.x;
         vA.y -= mA * impulseV.y;
         wA -= iA * (rAX * impulseV.y - rAY * impulseV.x);
         vB.x += mB * impulseV.x;
         vB.y += mB * impulseV.y;
         wB += iB * (rBX * impulseV.y - rBY * impulseV.x);
      }
      bA.angularVelocity = wA;
      bB.angularVelocity = wB;
   }
   FrictionJoint.prototype.SolvePositionConstraints = function (baumgarte) {
      if (baumgarte === undefined) baumgarte = 0;
      return true;
   }
   Box2D.inherit(FrictionJointDef, Box2D.Dynamics.Joints.JointDef);
   FrictionJointDef.prototype.__super = Box2D.Dynamics.Joints.JointDef.prototype;
   FrictionJointDef.FrictionJointDef = function () {
      Box2D.Dynamics.Joints.JointDef.JointDef.apply(this, arguments);
      this.localAnchorA = new Vec2();
      this.localAnchorB = new Vec2();
   };
   FrictionJointDef.prototype.FrictionJointDef = function () {
      this.__super.JointDef.call(this);
      this.type = Joint.e_frictionJoint;
      this.maxForce = 0.0;
      this.maxTorque = 0.0;
   }
   FrictionJointDef.prototype.Initialize = function (bA, bB, anchor) {
      this.bodyA = bA;
      this.bodyB = bB;
      this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchor));
      this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchor));
   }
   Box2D.inherit(GearJoint, Box2D.Dynamics.Joints.Joint);
   GearJoint.prototype.__super = Box2D.Dynamics.Joints.Joint.prototype;
   GearJoint.GearJoint = function () {
      Box2D.Dynamics.Joints.Joint.Joint.apply(this, arguments);
      this.groundAnchor1 = new Vec2();
      this.groundAnchor2 = new Vec2();
      this.localAnchor1 = new Vec2();
      this.localAnchor2 = new Vec2();
      this.J = new Jacobian();
   };
   GearJoint.prototype.GetAnchorA = function () {
      return this.bodyA.GetWorldPoint(this.localAnchor1);
   }
   GearJoint.prototype.GetAnchorB = function () {
      return this.bodyB.GetWorldPoint(this.localAnchor2);
   }
   GearJoint.prototype.GetReactionForce = function (inv_dt) {
      if (inv_dt === undefined) inv_dt = 0;
      return new Vec2(inv_dt * this.impulse * this.J.linearB.x, inv_dt * this.impulse * this.J.linearB.y);
   }
   GearJoint.prototype.GetReactionTorque = function (inv_dt) {
      if (inv_dt === undefined) inv_dt = 0;
      var tMat = this.bodyB.xf.R;
      var rX = this.localAnchor1.x - this.bodyB.sweep.localCenter.x;
      var rY = this.localAnchor1.y - this.bodyB.sweep.localCenter.y;
      var tX = tMat.col1.x * rX + tMat.col2.x * rY;
      rY = tMat.col1.y * rX + tMat.col2.y * rY;
      rX = tX;
      var PX = this.impulse * this.J.linearB.x;
      var PY = this.impulse * this.J.linearB.y;
      return inv_dt * (this.impulse * this.J.angularB - rX * PY + rY * PX);
   }
   GearJoint.prototype.GetRatio = function () {
      return this.ratio;
   }
   GearJoint.prototype.SetRatio = function (ratio) {
      if (ratio === undefined) ratio = 0;
      this.ratio = ratio;
   }
   GearJoint.prototype.GearJoint = function (def) {
      this.__super.Joint.call(this, def);
      var type1 = parseInt(def.joint1.type);
      var type2 = parseInt(def.joint2.type);
      this.revolute1 = null;
      this.prismatic1 = null;
      this.revolute2 = null;
      this.prismatic2 = null;
      var coordinate1 = 0;
      var coordinate2 = 0;
      this.ground1 = def.joint1.GetBodyA();
      this.bodyA = def.joint1.GetBodyB();
      if (type1 == Joint.e_revoluteJoint) {
         this.revolute1 = (def.joint1 instanceof RevoluteJoint ? def.joint1 : null);
         this.groundAnchor1.SetV(this.revolute1.localAnchor1);
         this.localAnchor1.SetV(this.revolute1.localAnchor2);
         coordinate1 = this.revolute1.GetJointAngle();
      }
      else {
         this.prismatic1 = (def.joint1 instanceof PrismaticJoint ? def.joint1 : null);
         this.groundAnchor1.SetV(this.prismatic1.localAnchor1);
         this.localAnchor1.SetV(this.prismatic1.localAnchor2);
         coordinate1 = this.prismatic1.GetJointTranslation();
      }
      this.ground2 = def.joint2.GetBodyA();
      this.bodyB = def.joint2.GetBodyB();
      if (type2 == Joint.e_revoluteJoint) {
         this.revolute2 = (def.joint2 instanceof RevoluteJoint ? def.joint2 : null);
         this.groundAnchor2.SetV(this.revolute2.localAnchor1);
         this.localAnchor2.SetV(this.revolute2.localAnchor2);
         coordinate2 = this.revolute2.GetJointAngle();
      }
      else {
         this.prismatic2 = (def.joint2 instanceof PrismaticJoint ? def.joint2 : null);
         this.groundAnchor2.SetV(this.prismatic2.localAnchor1);
         this.localAnchor2.SetV(this.prismatic2.localAnchor2);
         coordinate2 = this.prismatic2.GetJointTranslation();
      }
      this.ratio = def.ratio;
      this.constant = coordinate1 + this.ratio * coordinate2;
      this.impulse = 0.0;
   }
   GearJoint.prototype.InitVelocityConstraints = function (step) {
      var g1 = this.ground1;
      var g2 = this.ground2;
      var bA = this.bodyA;
      var bB = this.bodyB;
      var ugX = 0;
      var ugY = 0;
      var rX = 0;
      var rY = 0;
      var tMat;
      var tVec;
      var crug = 0;
      var tX = 0;
      var K = 0.0;
      this.J.SetZero();
      if (this.revolute1) {
         this.J.angularA = (-1.0);
         K += bA.invI;
      }
      else {
         tMat = g1.xf.R;
         tVec = this.prismatic1.localXAxis1;
         ugX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
         ugY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
         tMat = bA.xf.R;
         rX = this.localAnchor1.x - bA.sweep.localCenter.x;
         rY = this.localAnchor1.y - bA.sweep.localCenter.y;
         tX = tMat.col1.x * rX + tMat.col2.x * rY;
         rY = tMat.col1.y * rX + tMat.col2.y * rY;
         rX = tX;
         crug = rX * ugY - rY * ugX;
         this.J.linearA.Set((-ugX), (-ugY));
         this.J.angularA = (-crug);
         K += bA.invMass + bA.invI * crug * crug;
      }
      if (this.revolute2) {
         this.J.angularB = (-this.ratio);
         K += this.ratio * this.ratio * bB.invI;
      }
      else {
         tMat = g2.xf.R;
         tVec = this.prismatic2.localXAxis1;
         ugX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
         ugY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
         tMat = bB.xf.R;
         rX = this.localAnchor2.x - bB.sweep.localCenter.x;
         rY = this.localAnchor2.y - bB.sweep.localCenter.y;
         tX = tMat.col1.x * rX + tMat.col2.x * rY;
         rY = tMat.col1.y * rX + tMat.col2.y * rY;
         rX = tX;
         crug = rX * ugY - rY * ugX;
         this.J.linearB.Set((-this.ratio * ugX), (-this.ratio * ugY));
         this.J.angularB = (-this.ratio * crug);
         K += this.ratio * this.ratio * (bB.invMass + bB.invI * crug * crug);
      }
      this.mass = K > 0.0 ? 1.0 / K : 0.0;
      if (step.warmStarting) {
         bA.linearVelocity.x += bA.invMass * this.impulse * this.J.linearA.x;
         bA.linearVelocity.y += bA.invMass * this.impulse * this.J.linearA.y;
         bA.angularVelocity += bA.invI * this.impulse * this.J.angularA;
         bB.linearVelocity.x += bB.invMass * this.impulse * this.J.linearB.x;
         bB.linearVelocity.y += bB.invMass * this.impulse * this.J.linearB.y;
         bB.angularVelocity += bB.invI * this.impulse * this.J.angularB;
      }
      else {
         this.impulse = 0.0;
      }
   }
   GearJoint.prototype.SolveVelocityConstraints = function (step) {
      var bA = this.bodyA;
      var bB = this.bodyB;
      var Cdot = this.J.Compute(bA.linearVelocity, bA.angularVelocity, bB.linearVelocity, bB.angularVelocity);
      var impulse = (-this.mass * Cdot);
      this.impulse += impulse;
      bA.linearVelocity.x += bA.invMass * impulse * this.J.linearA.x;
      bA.linearVelocity.y += bA.invMass * impulse * this.J.linearA.y;
      bA.angularVelocity += bA.invI * impulse * this.J.angularA;
      bB.linearVelocity.x += bB.invMass * impulse * this.J.linearB.x;
      bB.linearVelocity.y += bB.invMass * impulse * this.J.linearB.y;
      bB.angularVelocity += bB.invI * impulse * this.J.angularB;
   }
   GearJoint.prototype.SolvePositionConstraints = function (baumgarte) {
      if (baumgarte === undefined) baumgarte = 0;
      var linearError = 0.0;
      var bA = this.bodyA;
      var bB = this.bodyB;
      var coordinate1 = 0;
      var coordinate2 = 0;
      if (this.revolute1) {
         coordinate1 = this.revolute1.GetJointAngle();
      }
      else {
         coordinate1 = this.prismatic1.GetJointTranslation();
      }
      if (this.revolute2) {
         coordinate2 = this.revolute2.GetJointAngle();
      }
      else {
         coordinate2 = this.prismatic2.GetJointTranslation();
      }
      var C = this.constant - (coordinate1 + this.ratio * coordinate2);
      var impulse = (-this.mass * C);
      bA.sweep.c.x += bA.invMass * impulse * this.J.linearA.x;
      bA.sweep.c.y += bA.invMass * impulse * this.J.linearA.y;
      bA.sweep.a += bA.invI * impulse * this.J.angularA;
      bB.sweep.c.x += bB.invMass * impulse * this.J.linearB.x;
      bB.sweep.c.y += bB.invMass * impulse * this.J.linearB.y;
      bB.sweep.a += bB.invI * impulse * this.J.angularB;
      bA.SynchronizeTransform();
      bB.SynchronizeTransform();
      return linearError < Settings._linearSlop;
   }
   Box2D.inherit(GearJointDef, Box2D.Dynamics.Joints.JointDef);
   GearJointDef.prototype.__super = Box2D.Dynamics.Joints.JointDef.prototype;
   GearJointDef.GearJointDef = function () {
      Box2D.Dynamics.Joints.JointDef.JointDef.apply(this, arguments);
   };
   GearJointDef.prototype.GearJointDef = function () {
      this.__super.JointDef.call(this);
      this.type = Joint.e_gearJoint;
      this.joint1 = null;
      this.joint2 = null;
      this.ratio = 1.0;
   }
   Jacobian.Jacobian = function () {
      this.linearA = new Vec2();
      this.linearB = new Vec2();
   };
   Jacobian.prototype.SetZero = function () {
      this.linearA.SetZero();
      this.angularA = 0.0;
      this.linearB.SetZero();
      this.angularB = 0.0;
   }
   Jacobian.prototype.Set = function (x1, a1, x2, a2) {
      if (a1 === undefined) a1 = 0;
      if (a2 === undefined) a2 = 0;
      this.linearA.SetV(x1);
      this.angularA = a1;
      this.linearB.SetV(x2);
      this.angularB = a2;
   }
   Jacobian.prototype.Compute = function (x1, a1, x2, a2) {
      if (a1 === undefined) a1 = 0;
      if (a2 === undefined) a2 = 0;
      return (this.linearA.x * x1.x + this.linearA.y * x1.y) + this.angularA * a1 + (this.linearB.x * x2.x + this.linearB.y * x2.y) + this.angularB * a2;
   }
   Joint.Joint = function () {
      this.edgeA = new JointEdge();
      this.edgeB = new JointEdge();
      this.localCenterA = new Vec2();
      this.localCenterB = new Vec2();
   };
   Joint.prototype.GetType = function () {
      return this.type;
   }
   Joint.prototype.GetAnchorA = function () {
      return null;
   }
   Joint.prototype.GetAnchorB = function () {
      return null;
   }
   Joint.prototype.GetReactionForce = function (inv_dt) {
      if (inv_dt === undefined) inv_dt = 0;
      return null;
   }
   Joint.prototype.GetReactionTorque = function (inv_dt) {
      if (inv_dt === undefined) inv_dt = 0;
      return 0.0;
   }
   Joint.prototype.GetBodyA = function () {
      return this.bodyA;
   }
   Joint.prototype.GetBodyB = function () {
      return this.bodyB;
   }
   Joint.prototype.GetNext = function () {
      return this.next;
   }
   Joint.prototype.GetUserData = function () {
      return this.userData;
   }
   Joint.prototype.SetUserData = function (data) {
      this.userData = data;
   }
   Joint.prototype.IsActive = function () {
      return this.bodyA.IsActive() && this.bodyB.IsActive();
   }
   Joint.Create = function (def, allocator) {
      var joint = null;
      switch (def.type) {
      case Joint.e_distanceJoint:
         {
            joint = new DistanceJoint((def instanceof DistanceJointDef ? def : null));
         }
         break;
      case Joint.e_mouseJoint:
         {
            joint = new MouseJoint((def instanceof MouseJointDef ? def : null));
         }
         break;
      case Joint.e_prismaticJoint:
         {
            joint = new PrismaticJoint((def instanceof PrismaticJointDef ? def : null));
         }
         break;
      case Joint.e_revoluteJoint:
         {
            joint = new RevoluteJoint((def instanceof RevoluteJointDef ? def : null));
         }
         break;
      case Joint.e_pulleyJoint:
         {
            joint = new PulleyJoint((def instanceof PulleyJointDef ? def : null));
         }
         break;
      case Joint.e_gearJoint:
         {
            joint = new GearJoint((def instanceof GearJointDef ? def : null));
         }
         break;
      case Joint.e_lineJoint:
         {
            joint = new LineJoint((def instanceof LineJointDef ? def : null));
         }
         break;
      case Joint.e_weldJoint:
         {
            joint = new WeldJoint((def instanceof WeldJointDef ? def : null));
         }
         break;
      case Joint.e_frictionJoint:
         {
            joint = new FrictionJoint((def instanceof FrictionJointDef ? def : null));
         }
         break;
      default:
         break;
      }
      return joint;
   }
   Joint.Destroy = function (joint, allocator) {}
   Joint.prototype.Joint = function (def) {
      Settings.Assert(def.bodyA != def.bodyB);
      this.type = def.type;
      this.prev = null;
      this.next = null;
      this.bodyA = def.bodyA;
      this.bodyB = def.bodyB;
      this.collideConnected = def.collideConnected;
      this.islandFlag = false;
      this.userData = def.userData;
   }
   Joint.prototype.InitVelocityConstraints = function (step) {}
   Joint.prototype.SolveVelocityConstraints = function (step) {}
   Joint.prototype.FinalizeVelocityConstraints = function () {}
   Joint.prototype.SolvePositionConstraints = function (baumgarte) {
      if (baumgarte === undefined) baumgarte = 0;
      return false;
   }
   Box2D.postDefs.push(function () {
      Box2D.Dynamics.Joints.Joint.e_unknownJoint = 0;
      Box2D.Dynamics.Joints.Joint.e_revoluteJoint = 1;
      Box2D.Dynamics.Joints.Joint.e_prismaticJoint = 2;
      Box2D.Dynamics.Joints.Joint.e_distanceJoint = 3;
      Box2D.Dynamics.Joints.Joint.e_pulleyJoint = 4;
      Box2D.Dynamics.Joints.Joint.e_mouseJoint = 5;
      Box2D.Dynamics.Joints.Joint.e_gearJoint = 6;
      Box2D.Dynamics.Joints.Joint.e_lineJoint = 7;
      Box2D.Dynamics.Joints.Joint.e_weldJoint = 8;
      Box2D.Dynamics.Joints.Joint.e_frictionJoint = 9;
      Box2D.Dynamics.Joints.Joint.e_inactiveLimit = 0;
      Box2D.Dynamics.Joints.Joint.e_atLowerLimit = 1;
      Box2D.Dynamics.Joints.Joint.e_atUpperLimit = 2;
      Box2D.Dynamics.Joints.Joint.e_equalLimits = 3;
   });
   JointDef.JointDef = function () {};
   JointDef.prototype.JointDef = function () {
      this.type = Joint.e_unknownJoint;
      this.userData = null;
      this.bodyA = null;
      this.bodyB = null;
      this.collideConnected = false;
   }
   JointEdge.JointEdge = function () {};
   Box2D.inherit(LineJoint, Box2D.Dynamics.Joints.Joint);
   LineJoint.prototype.__super = Box2D.Dynamics.Joints.Joint.prototype;
   LineJoint.LineJoint = function () {
      Box2D.Dynamics.Joints.Joint.Joint.apply(this, arguments);
      this.localAnchor1 = new Vec2();
      this.localAnchor2 = new Vec2();
      this.localXAxis1 = new Vec2();
      this.localYAxis1 = new Vec2();
      this.axis = new Vec2();
      this.perp = new Vec2();
      this.K = new Mat22();
      this.impulse = new Vec2();
   };
   LineJoint.prototype.GetAnchorA = function () {
      return this.bodyA.GetWorldPoint(this.localAnchor1);
   }
   LineJoint.prototype.GetAnchorB = function () {
      return this.bodyB.GetWorldPoint(this.localAnchor2);
   }
   LineJoint.prototype.GetReactionForce = function (inv_dt) {
      if (inv_dt === undefined) inv_dt = 0;
      return new Vec2(inv_dt * (this.impulse.x * this.perp.x + (this.motorImpulse + this.impulse.y) * this.axis.x), inv_dt * (this.impulse.x * this.perp.y + (this.motorImpulse + this.impulse.y) * this.axis.y));
   }
   LineJoint.prototype.GetReactionTorque = function (inv_dt) {
      if (inv_dt === undefined) inv_dt = 0;
      return inv_dt * this.impulse.y;
   }
   LineJoint.prototype.GetJointTranslation = function () {
      var bA = this.bodyA;
      var bB = this.bodyB;
      var tMat;
      var p1 = bA.GetWorldPoint(this.localAnchor1);
      var p2 = bB.GetWorldPoint(this.localAnchor2);
      var dX = p2.x - p1.x;
      var dY = p2.y - p1.y;
      var axis = bA.GetWorldVector(this.localXAxis1);
      var translation = axis.x * dX + axis.y * dY;
      return translation;
   }
   LineJoint.prototype.GetJointSpeed = function () {
      var bA = this.bodyA;
      var bB = this.bodyB;
      var tMat;
      tMat = bA.xf.R;
      var r1X = this.localAnchor1.x - bA.sweep.localCenter.x;
      var r1Y = this.localAnchor1.y - bA.sweep.localCenter.y;
      var tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
      r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
      r1X = tX;
      tMat = bB.xf.R;
      var r2X = this.localAnchor2.x - bB.sweep.localCenter.x;
      var r2Y = this.localAnchor2.y - bB.sweep.localCenter.y;
      tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
      r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
      r2X = tX;
      var p1X = bA.sweep.c.x + r1X;
      var p1Y = bA.sweep.c.y + r1Y;
      var p2X = bB.sweep.c.x + r2X;
      var p2Y = bB.sweep.c.y + r2Y;
      var dX = p2X - p1X;
      var dY = p2Y - p1Y;
      var axis = bA.GetWorldVector(this.localXAxis1);
      var v1 = bA.linearVelocity;
      var v2 = bB.linearVelocity;
      var w1 = bA.angularVelocity;
      var w2 = bB.angularVelocity;
      var speed = (dX * ((-w1 * axis.y)) + dY * (w1 * axis.x)) + (axis.x * (((v2.x + ((-w2 * r2Y))) - v1.x) - ((-w1 * r1Y))) + axis.y * (((v2.y + (w2 * r2X)) - v1.y) - (w1 * r1X)));
      return speed;
   }
   LineJoint.prototype.IsLimitEnabled = function () {
      return this.enableLimit;
   }
   LineJoint.prototype.EnableLimit = function (flag) {
      this.bodyA.SetAwake(true);
      this.bodyB.SetAwake(true);
      this.enableLimit = flag;
   }
   LineJoint.prototype.GetLowerLimit = function () {
      return this.lowerTranslation;
   }
   LineJoint.prototype.GetUpperLimit = function () {
      return this.upperTranslation;
   }
   LineJoint.prototype.SetLimits = function (lower, upper) {
      if (lower === undefined) lower = 0;
      if (upper === undefined) upper = 0;
      this.bodyA.SetAwake(true);
      this.bodyB.SetAwake(true);
      this.lowerTranslation = lower;
      this.upperTranslation = upper;
   }
   LineJoint.prototype.IsMotorEnabled = function () {
      return this.enableMotor;
   }
   LineJoint.prototype.EnableMotor = function (flag) {
      this.bodyA.SetAwake(true);
      this.bodyB.SetAwake(true);
      this.enableMotor = flag;
   }
   LineJoint.prototype.SetMotorSpeed = function (speed) {
      if (speed === undefined) speed = 0;
      this.bodyA.SetAwake(true);
      this.bodyB.SetAwake(true);
      this.motorSpeed = speed;
   }
   LineJoint.prototype.GetMotorSpeed = function () {
      return this.motorSpeed;
   }
   LineJoint.prototype.SetMaxMotorForce = function (force) {
      if (force === undefined) force = 0;
      this.bodyA.SetAwake(true);
      this.bodyB.SetAwake(true);
      this.maxMotorForce = force;
   }
   LineJoint.prototype.GetMaxMotorForce = function () {
      return this.maxMotorForce;
   }
   LineJoint.prototype.GetMotorForce = function () {
      return this.motorImpulse;
   }
   LineJoint.prototype.LineJoint = function (def) {
      this.__super.Joint.call(this, def);
      var tMat;
      var tX = 0;
      var tY = 0;
      this.localAnchor1.SetV(def.localAnchorA);
      this.localAnchor2.SetV(def.localAnchorB);
      this.localXAxis1.SetV(def.localAxisA);
      this.localYAxis1.x = (-this.localXAxis1.y);
      this.localYAxis1.y = this.localXAxis1.x;
      this.impulse.SetZero();
      this.motorMass = 0.0;
      this.motorImpulse = 0.0;
      this.lowerTranslation = def.lowerTranslation;
      this.upperTranslation = def.upperTranslation;
      this.maxMotorForce = def.maxMotorForce;
      this.motorSpeed = def.motorSpeed;
      this.enableLimit = def.enableLimit;
      this.enableMotor = def.enableMotor;
      this.limitState = Joint.e_inactiveLimit;
      this.axis.SetZero();
      this.perp.SetZero();
   }
   LineJoint.prototype.InitVelocityConstraints = function (step) {
      var bA = this.bodyA;
      var bB = this.bodyB;
      var tMat;
      var tX = 0;
      this.localCenterA.SetV(bA.GetLocalCenter());
      this.localCenterB.SetV(bB.GetLocalCenter());
      var xf1 = bA.GetTransform();
      var xf2 = bB.GetTransform();
      tMat = bA.xf.R;
      var r1X = this.localAnchor1.x - this.localCenterA.x;
      var r1Y = this.localAnchor1.y - this.localCenterA.y;
      tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
      r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
      r1X = tX;
      tMat = bB.xf.R;
      var r2X = this.localAnchor2.x - this.localCenterB.x;
      var r2Y = this.localAnchor2.y - this.localCenterB.y;
      tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
      r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
      r2X = tX;
      var dX = bB.sweep.c.x + r2X - bA.sweep.c.x - r1X;
      var dY = bB.sweep.c.y + r2Y - bA.sweep.c.y - r1Y;
      this.invMassA = bA.invMass;
      this.invMassB = bB.invMass;
      this.invIA = bA.invI;
      this.invIB = bB.invI; {
         this.axis.SetV(Math.MulMV(xf1.R, this.localXAxis1));
         this.a1 = (dX + r1X) * this.axis.y - (dY + r1Y) * this.axis.x;
         this.a2 = r2X * this.axis.y - r2Y * this.axis.x;
         this.motorMass = this.invMassA + this.invMassB + this.invIA * this.a1 * this.a1 + this.invIB * this.a2 * this.a2;
         this.motorMass = this.motorMass > Number.MIN_VALUE ? 1.0 / this.motorMass : 0.0;
      } {
         this.perp.SetV(Math.MulMV(xf1.R, this.localYAxis1));
         this.s1 = (dX + r1X) * this.perp.y - (dY + r1Y) * this.perp.x;
         this.s2 = r2X * this.perp.y - r2Y * this.perp.x;
         var m1 = this.invMassA;
         var m2 = this.invMassB;
         var i1 = this.invIA;
         var i2 = this.invIB;
         this.K.col1.x = m1 + m2 + i1 * this.s1 * this.s1 + i2 * this.s2 * this.s2;
         this.K.col1.y = i1 * this.s1 * this.a1 + i2 * this.s2 * this.a2;
         this.K.col2.x = this.K.col1.y;
         this.K.col2.y = m1 + m2 + i1 * this.a1 * this.a1 + i2 * this.a2 * this.a2;
      }
      if (this.enableLimit) {
         var jointTransition = this.axis.x * dX + this.axis.y * dY;
         if (Math.Abs(this.upperTranslation - this.lowerTranslation) < 2.0 * Settings._linearSlop) {
            this.limitState = Joint.e_equalLimits;
         }
         else if (jointTransition <= this.lowerTranslation) {
            if (this.limitState != Joint.e_atLowerLimit) {
               this.limitState = Joint.e_atLowerLimit;
               this.impulse.y = 0.0;
            }
         }
         else if (jointTransition >= this.upperTranslation) {
            if (this.limitState != Joint.e_atUpperLimit) {
               this.limitState = Joint.e_atUpperLimit;
               this.impulse.y = 0.0;
            }
         }
         else {
            this.limitState = Joint.e_inactiveLimit;
            this.impulse.y = 0.0;
         }
      }
      else {
         this.limitState = Joint.e_inactiveLimit;
      }
      if (this.enableMotor == false) {
         this.motorImpulse = 0.0;
      }
      if (step.warmStarting) {
         this.impulse.x *= step.dtRatio;
         this.impulse.y *= step.dtRatio;
         this.motorImpulse *= step.dtRatio;
         var PX = this.impulse.x * this.perp.x + (this.motorImpulse + this.impulse.y) * this.axis.x;
         var PY = this.impulse.x * this.perp.y + (this.motorImpulse + this.impulse.y) * this.axis.y;
         var L1 = this.impulse.x * this.s1 + (this.motorImpulse + this.impulse.y) * this.a1;
         var L2 = this.impulse.x * this.s2 + (this.motorImpulse + this.impulse.y) * this.a2;
         bA.linearVelocity.x -= this.invMassA * PX;
         bA.linearVelocity.y -= this.invMassA * PY;
         bA.angularVelocity -= this.invIA * L1;
         bB.linearVelocity.x += this.invMassB * PX;
         bB.linearVelocity.y += this.invMassB * PY;
         bB.angularVelocity += this.invIB * L2;
      }
      else {
         this.impulse.SetZero();
         this.motorImpulse = 0.0;
      }
   }
   LineJoint.prototype.SolveVelocityConstraints = function (step) {
      var bA = this.bodyA;
      var bB = this.bodyB;
      var v1 = bA.linearVelocity;
      var w1 = bA.angularVelocity;
      var v2 = bB.linearVelocity;
      var w2 = bB.angularVelocity;
      var PX = 0;
      var PY = 0;
      var L1 = 0;
      var L2 = 0;
      if (this.enableMotor && this.limitState != Joint.e_equalLimits) {
         var Cdot = this.axis.x * (v2.x - v1.x) + this.axis.y * (v2.y - v1.y) + this.a2 * w2 - this.a1 * w1;
         var impulse = this.motorMass * (this.motorSpeed - Cdot);
         var oldImpulse = this.motorImpulse;
         var maxImpulse = step.dt * this.maxMotorForce;
         this.motorImpulse = Math.Clamp(this.motorImpulse + impulse, (-maxImpulse), maxImpulse);
         impulse = this.motorImpulse - oldImpulse;
         PX = impulse * this.axis.x;
         PY = impulse * this.axis.y;
         L1 = impulse * this.a1;
         L2 = impulse * this.a2;
         v1.x -= this.invMassA * PX;
         v1.y -= this.invMassA * PY;
         w1 -= this.invIA * L1;
         v2.x += this.invMassB * PX;
         v2.y += this.invMassB * PY;
         w2 += this.invIB * L2;
      }
      var Cdot1 = this.perp.x * (v2.x - v1.x) + this.perp.y * (v2.y - v1.y) + this.s2 * w2 - this.s1 * w1;
      if (this.enableLimit && this.limitState != Joint.e_inactiveLimit) {
         var Cdot2 = this.axis.x * (v2.x - v1.x) + this.axis.y * (v2.y - v1.y) + this.a2 * w2 - this.a1 * w1;
         var f1 = this.impulse.Copy();
         var df = this.K.Solve(new Vec2(), (-Cdot1), (-Cdot2));
         this.impulse.Add(df);
         if (this.limitState == Joint.e_atLowerLimit) {
            this.impulse.y = Math.Max(this.impulse.y, 0.0);
         }
         else if (this.limitState == Joint.e_atUpperLimit) {
            this.impulse.y = Math.Min(this.impulse.y, 0.0);
         }
         var b = (-Cdot1) - (this.impulse.y - f1.y) * this.K.col2.x;
         var f2r = 0;
         if (this.K.col1.x != 0.0) {
            f2r = b / this.K.col1.x + f1.x;
         }
         else {
            f2r = f1.x;
         }
         this.impulse.x = f2r;
         df.x = this.impulse.x - f1.x;
         df.y = this.impulse.y - f1.y;
         PX = df.x * this.perp.x + df.y * this.axis.x;
         PY = df.x * this.perp.y + df.y * this.axis.y;
         L1 = df.x * this.s1 + df.y * this.a1;
         L2 = df.x * this.s2 + df.y * this.a2;
         v1.x -= this.invMassA * PX;
         v1.y -= this.invMassA * PY;
         w1 -= this.invIA * L1;
         v2.x += this.invMassB * PX;
         v2.y += this.invMassB * PY;
         w2 += this.invIB * L2;
      }
      else {
         var df2 = 0;
         if (this.K.col1.x != 0.0) {
            df2 = ((-Cdot1)) / this.K.col1.x;
         }
         else {
            df2 = 0.0;
         }
         this.impulse.x += df2;
         PX = df2 * this.perp.x;
         PY = df2 * this.perp.y;
         L1 = df2 * this.s1;
         L2 = df2 * this.s2;
         v1.x -= this.invMassA * PX;
         v1.y -= this.invMassA * PY;
         w1 -= this.invIA * L1;
         v2.x += this.invMassB * PX;
         v2.y += this.invMassB * PY;
         w2 += this.invIB * L2;
      }
      bA.linearVelocity.SetV(v1);
      bA.angularVelocity = w1;
      bB.linearVelocity.SetV(v2);
      bB.angularVelocity = w2;
   }
   LineJoint.prototype.SolvePositionConstraints = function (baumgarte) {
      if (baumgarte === undefined) baumgarte = 0;
      var limitC = 0;
      var oldLimitImpulse = 0;
      var bA = this.bodyA;
      var bB = this.bodyB;
      var c1 = bA.sweep.c;
      var a1 = bA.sweep.a;
      var c2 = bB.sweep.c;
      var a2 = bB.sweep.a;
      var tMat;
      var tX = 0;
      var m1 = 0;
      var m2 = 0;
      var i1 = 0;
      var i2 = 0;
      var linearError = 0.0;
      var angularError = 0.0;
      var active = false;
      var C2 = 0.0;
      var R1 = Mat22.FromAngle(a1);
      var R2 = Mat22.FromAngle(a2);
      tMat = R1;
      var r1X = this.localAnchor1.x - this.localCenterA.x;
      var r1Y = this.localAnchor1.y - this.localCenterA.y;
      tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
      r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
      r1X = tX;
      tMat = R2;
      var r2X = this.localAnchor2.x - this.localCenterB.x;
      var r2Y = this.localAnchor2.y - this.localCenterB.y;
      tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
      r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
      r2X = tX;
      var dX = c2.x + r2X - c1.x - r1X;
      var dY = c2.y + r2Y - c1.y - r1Y;
      if (this.enableLimit) {
         this.axis = Math.MulMV(R1, this.localXAxis1);
         this.a1 = (dX + r1X) * this.axis.y - (dY + r1Y) * this.axis.x;
         this.a2 = r2X * this.axis.y - r2Y * this.axis.x;
         var translation = this.axis.x * dX + this.axis.y * dY;
         if (Math.Abs(this.upperTranslation - this.lowerTranslation) < 2.0 * Settings._linearSlop) {
            C2 = Math.Clamp(translation, (-Settings._maxLinearCorrection), Settings._maxLinearCorrection);
            linearError = Math.Abs(translation);
            active = true;
         }
         else if (translation <= this.lowerTranslation) {
            C2 = Math.Clamp(translation - this.lowerTranslation + Settings._linearSlop, (-Settings._maxLinearCorrection), 0.0);
            linearError = this.lowerTranslation - translation;
            active = true;
         }
         else if (translation >= this.upperTranslation) {
            C2 = Math.Clamp(translation - this.upperTranslation + Settings._linearSlop, 0.0, Settings._maxLinearCorrection);
            linearError = translation - this.upperTranslation;
            active = true;
         }
      }
      this.perp = Math.MulMV(R1, this.localYAxis1);
      this.s1 = (dX + r1X) * this.perp.y - (dY + r1Y) * this.perp.x;
      this.s2 = r2X * this.perp.y - r2Y * this.perp.x;
      var impulse = new Vec2();
      var C1 = this.perp.x * dX + this.perp.y * dY;
      linearError = Math.Max(linearError, Math.Abs(C1));
      angularError = 0.0;
      if (active) {
         m1 = this.invMassA;
         m2 = this.invMassB;
         i1 = this.invIA;
         i2 = this.invIB;
         this.K.col1.x = m1 + m2 + i1 * this.s1 * this.s1 + i2 * this.s2 * this.s2;
         this.K.col1.y = i1 * this.s1 * this.a1 + i2 * this.s2 * this.a2;
         this.K.col2.x = this.K.col1.y;
         this.K.col2.y = m1 + m2 + i1 * this.a1 * this.a1 + i2 * this.a2 * this.a2;
         this.K.Solve(impulse, (-C1), (-C2));
      }
      else {
         m1 = this.invMassA;
         m2 = this.invMassB;
         i1 = this.invIA;
         i2 = this.invIB;
         var k11 = m1 + m2 + i1 * this.s1 * this.s1 + i2 * this.s2 * this.s2;
         var impulse1 = 0;
         if (k11 != 0.0) {
            impulse1 = ((-C1)) / k11;
         }
         else {
            impulse1 = 0.0;
         }
         impulse.x = impulse1;
         impulse.y = 0.0;
      }
      var PX = impulse.x * this.perp.x + impulse.y * this.axis.x;
      var PY = impulse.x * this.perp.y + impulse.y * this.axis.y;
      var L1 = impulse.x * this.s1 + impulse.y * this.a1;
      var L2 = impulse.x * this.s2 + impulse.y * this.a2;
      c1.x -= this.invMassA * PX;
      c1.y -= this.invMassA * PY;
      a1 -= this.invIA * L1;
      c2.x += this.invMassB * PX;
      c2.y += this.invMassB * PY;
      a2 += this.invIB * L2;
      bA.sweep.a = a1;
      bB.sweep.a = a2;
      bA.SynchronizeTransform();
      bB.SynchronizeTransform();
      return linearError <= Settings._linearSlop && angularError <= Settings._angularSlop;
   }
   Box2D.inherit(LineJointDef, Box2D.Dynamics.Joints.JointDef);
   LineJointDef.prototype.__super = Box2D.Dynamics.Joints.JointDef.prototype;
   LineJointDef.LineJointDef = function () {
      Box2D.Dynamics.Joints.JointDef.JointDef.apply(this, arguments);
      this.localAnchorA = new Vec2();
      this.localAnchorB = new Vec2();
      this.localAxisA = new Vec2();
   };
   LineJointDef.prototype.LineJointDef = function () {
      this.__super.JointDef.call(this);
      this.type = Joint.e_lineJoint;
      this.localAxisA.Set(1.0, 0.0);
      this.enableLimit = false;
      this.lowerTranslation = 0.0;
      this.upperTranslation = 0.0;
      this.enableMotor = false;
      this.maxMotorForce = 0.0;
      this.motorSpeed = 0.0;
   }
   LineJointDef.prototype.Initialize = function (bA, bB, anchor, axis) {
      this.bodyA = bA;
      this.bodyB = bB;
      this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
      this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
      this.localAxisA = this.bodyA.GetLocalVector(axis);
   }
   Box2D.inherit(MouseJoint, Box2D.Dynamics.Joints.Joint);
   MouseJoint.prototype.__super = Box2D.Dynamics.Joints.Joint.prototype;
   MouseJoint.MouseJoint = function () {
      Box2D.Dynamics.Joints.Joint.Joint.apply(this, arguments);
      this.K = new Mat22();
      this.K1 = new Mat22();
      this.K2 = new Mat22();
      this.localAnchor = new Vec2();
      this.target = new Vec2();
      this.impulse = new Vec2();
      this.mass = new Mat22();
      this.C = new Vec2();
   };
   MouseJoint.prototype.GetAnchorA = function () {
      return this.target;
   }
   MouseJoint.prototype.GetAnchorB = function () {
      return this.bodyB.GetWorldPoint(this.localAnchor);
   }
   MouseJoint.prototype.GetReactionForce = function (inv_dt) {
      if (inv_dt === undefined) inv_dt = 0;
      return new Vec2(inv_dt * this.impulse.x, inv_dt * this.impulse.y);
   }
   MouseJoint.prototype.GetReactionTorque = function (inv_dt) {
      if (inv_dt === undefined) inv_dt = 0;
      return 0.0;
   }
   MouseJoint.prototype.GetTarget = function () {
      return this.target;
   }
   MouseJoint.prototype.SetTarget = function (target) {
      if (this.bodyB.IsAwake() == false) {
         this.bodyB.SetAwake(true);
      }
      this.target = target;
   }
   MouseJoint.prototype.GetMaxForce = function () {
      return this.maxForce;
   }
   MouseJoint.prototype.SetMaxForce = function (maxForce) {
      if (maxForce === undefined) maxForce = 0;
      this.maxForce = maxForce;
   }
   MouseJoint.prototype.GetFrequency = function () {
      return this.frequencyHz;
   }
   MouseJoint.prototype.SetFrequency = function (hz) {
      if (hz === undefined) hz = 0;
      this.frequencyHz = hz;
   }
   MouseJoint.prototype.GetDampingRatio = function () {
      return this.dampingRatio;
   }
   MouseJoint.prototype.SetDampingRatio = function (ratio) {
      if (ratio === undefined) ratio = 0;
      this.dampingRatio = ratio;
   }
   MouseJoint.prototype.MouseJoint = function (def) {
      this.__super.Joint.call(this, def);
      this.target.SetV(def.target);
      var tX = this.target.x - this.bodyB.xf.position.x;
      var tY = this.target.y - this.bodyB.xf.position.y;
      var tMat = this.bodyB.xf.R;
      this.localAnchor.x = (tX * tMat.col1.x + tY * tMat.col1.y);
      this.localAnchor.y = (tX * tMat.col2.x + tY * tMat.col2.y);
      this.maxForce = def.maxForce;
      this.impulse.SetZero();
      this.frequencyHz = def.frequencyHz;
      this.dampingRatio = def.dampingRatio;
      this.beta = 0.0;
      this.gamma = 0.0;
   }
   MouseJoint.prototype.InitVelocityConstraints = function (step) {
      var b = this.bodyB;
      var mass = b.GetMass();
      var omega = 2.0 * Math.PI * this.frequencyHz;
      var d = 2.0 * mass * this.dampingRatio * omega;
      var k = mass * omega * omega;
      this.gamma = step.dt * (d + step.dt * k);
      this.gamma = this.gamma != 0 ? 1 / this.gamma : 0.0;
      this.beta = step.dt * k * this.gamma;
      var tMat;tMat = b.xf.R;
      var rX = this.localAnchor.x - b.sweep.localCenter.x;
      var rY = this.localAnchor.y - b.sweep.localCenter.y;
      var tX = (tMat.col1.x * rX + tMat.col2.x * rY);rY = (tMat.col1.y * rX + tMat.col2.y * rY);
      rX = tX;
      var invMass = b.invMass;
      var invI = b.invI;this.K1.col1.x = invMass;
      this.K1.col2.x = 0.0;
      this.K1.col1.y = 0.0;
      this.K1.col2.y = invMass;
      this.K2.col1.x = invI * rY * rY;
      this.K2.col2.x = (-invI * rX * rY);
      this.K2.col1.y = (-invI * rX * rY);
      this.K2.col2.y = invI * rX * rX;
      this.K.SetM(this.K1);
      this.K.AddM(this.K2);
      this.K.col1.x += this.gamma;
      this.K.col2.y += this.gamma;
      this.K.GetInverse(this.mass);
      this.C.x = b.sweep.c.x + rX - this.target.x;
      this.C.y = b.sweep.c.y + rY - this.target.y;
      b.angularVelocity *= 0.98;
      this.impulse.x *= step.dtRatio;
      this.impulse.y *= step.dtRatio;
      b.linearVelocity.x += invMass * this.impulse.x;
      b.linearVelocity.y += invMass * this.impulse.y;
      b.angularVelocity += invI * (rX * this.impulse.y - rY * this.impulse.x);
   }
   MouseJoint.prototype.SolveVelocityConstraints = function (step) {
      var b = this.bodyB;
      var tMat;
      var tX = 0;
      var tY = 0;
      tMat = b.xf.R;
      var rX = this.localAnchor.x - b.sweep.localCenter.x;
      var rY = this.localAnchor.y - b.sweep.localCenter.y;
      tX = (tMat.col1.x * rX + tMat.col2.x * rY);
      rY = (tMat.col1.y * rX + tMat.col2.y * rY);
      rX = tX;
      var CdotX = b.linearVelocity.x + ((-b.angularVelocity * rY));
      var CdotY = b.linearVelocity.y + (b.angularVelocity * rX);
      tMat = this.mass;
      tX = CdotX + this.beta * this.C.x + this.gamma * this.impulse.x;
      tY = CdotY + this.beta * this.C.y + this.gamma * this.impulse.y;
      var impulseX = (-(tMat.col1.x * tX + tMat.col2.x * tY));
      var impulseY = (-(tMat.col1.y * tX + tMat.col2.y * tY));
      var oldImpulseX = this.impulse.x;
      var oldImpulseY = this.impulse.y;
      this.impulse.x += impulseX;
      this.impulse.y += impulseY;
      var maxImpulse = step.dt * this.maxForce;
      if (this.impulse.LengthSquared() > maxImpulse * maxImpulse) {
         this.impulse.Multiply(maxImpulse / this.impulse.Length());
      }
      impulseX = this.impulse.x - oldImpulseX;
      impulseY = this.impulse.y - oldImpulseY;
      b.linearVelocity.x += b.invMass * impulseX;
      b.linearVelocity.y += b.invMass * impulseY;
      b.angularVelocity += b.invI * (rX * impulseY - rY * impulseX);
   }
   MouseJoint.prototype.SolvePositionConstraints = function (baumgarte) {
      if (baumgarte === undefined) baumgarte = 0;
      return true;
   }
   Box2D.inherit(MouseJointDef, Box2D.Dynamics.Joints.JointDef);
   MouseJointDef.prototype.__super = Box2D.Dynamics.Joints.JointDef.prototype;
   MouseJointDef.MouseJointDef = function () {
      Box2D.Dynamics.Joints.JointDef.JointDef.apply(this, arguments);
      this.target = new Vec2();
   };
   MouseJointDef.prototype.MouseJointDef = function () {
      this.__super.JointDef.call(this);
      this.type = Joint.e_mouseJoint;
      this.maxForce = 0.0;
      this.frequencyHz = 5.0;
      this.dampingRatio = 0.7;
   }
   Box2D.inherit(PrismaticJoint, Box2D.Dynamics.Joints.Joint);
   PrismaticJoint.prototype.__super = Box2D.Dynamics.Joints.Joint.prototype;
   PrismaticJoint.PrismaticJoint = function () {
      Box2D.Dynamics.Joints.Joint.Joint.apply(this, arguments);
      this.localAnchor1 = new Vec2();
      this.localAnchor2 = new Vec2();
      this.localXAxis1 = new Vec2();
      this.localYAxis1 = new Vec2();
      this.axis = new Vec2();
      this.perp = new Vec2();
      this.K = new Mat33();
      this.impulse = new Vec3();
   };
   PrismaticJoint.prototype.GetAnchorA = function () {
      return this.bodyA.GetWorldPoint(this.localAnchor1);
   }
   PrismaticJoint.prototype.GetAnchorB = function () {
      return this.bodyB.GetWorldPoint(this.localAnchor2);
   }
   PrismaticJoint.prototype.GetReactionForce = function (inv_dt) {
      if (inv_dt === undefined) inv_dt = 0;
      return new Vec2(inv_dt * (this.impulse.x * this.perp.x + (this.motorImpulse + this.impulse.z) * this.axis.x), inv_dt * (this.impulse.x * this.perp.y + (this.motorImpulse + this.impulse.z) * this.axis.y));
   }
   PrismaticJoint.prototype.GetReactionTorque = function (inv_dt) {
      if (inv_dt === undefined) inv_dt = 0;
      return inv_dt * this.impulse.y;
   }
   PrismaticJoint.prototype.GetJointTranslation = function () {
      var bA = this.bodyA;
      var bB = this.bodyB;
      var tMat;
      var p1 = bA.GetWorldPoint(this.localAnchor1);
      var p2 = bB.GetWorldPoint(this.localAnchor2);
      var dX = p2.x - p1.x;
      var dY = p2.y - p1.y;
      var axis = bA.GetWorldVector(this.localXAxis1);
      var translation = axis.x * dX + axis.y * dY;
      return translation;
   }
   PrismaticJoint.prototype.GetJointSpeed = function () {
      var bA = this.bodyA;
      var bB = this.bodyB;
      var tMat;
      tMat = bA.xf.R;
      var r1X = this.localAnchor1.x - bA.sweep.localCenter.x;
      var r1Y = this.localAnchor1.y - bA.sweep.localCenter.y;
      var tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
      r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
      r1X = tX;
      tMat = bB.xf.R;
      var r2X = this.localAnchor2.x - bB.sweep.localCenter.x;
      var r2Y = this.localAnchor2.y - bB.sweep.localCenter.y;
      tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
      r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
      r2X = tX;
      var p1X = bA.sweep.c.x + r1X;
      var p1Y = bA.sweep.c.y + r1Y;
      var p2X = bB.sweep.c.x + r2X;
      var p2Y = bB.sweep.c.y + r2Y;
      var dX = p2X - p1X;
      var dY = p2Y - p1Y;
      var axis = bA.GetWorldVector(this.localXAxis1);
      var v1 = bA.linearVelocity;
      var v2 = bB.linearVelocity;
      var w1 = bA.angularVelocity;
      var w2 = bB.angularVelocity;
      var speed = (dX * ((-w1 * axis.y)) + dY * (w1 * axis.x)) + (axis.x * (((v2.x + ((-w2 * r2Y))) - v1.x) - ((-w1 * r1Y))) + axis.y * (((v2.y + (w2 * r2X)) - v1.y) - (w1 * r1X)));
      return speed;
   }
   PrismaticJoint.prototype.IsLimitEnabled = function () {
      return this.enableLimit;
   }
   PrismaticJoint.prototype.EnableLimit = function (flag) {
      this.bodyA.SetAwake(true);
      this.bodyB.SetAwake(true);
      this.enableLimit = flag;
   }
   PrismaticJoint.prototype.GetLowerLimit = function () {
      return this.lowerTranslation;
   }
   PrismaticJoint.prototype.GetUpperLimit = function () {
      return this.upperTranslation;
   }
   PrismaticJoint.prototype.SetLimits = function (lower, upper) {
      if (lower === undefined) lower = 0;
      if (upper === undefined) upper = 0;
      this.bodyA.SetAwake(true);
      this.bodyB.SetAwake(true);
      this.lowerTranslation = lower;
      this.upperTranslation = upper;
   }
   PrismaticJoint.prototype.IsMotorEnabled = function () {
      return this.enableMotor;
   }
   PrismaticJoint.prototype.EnableMotor = function (flag) {
      this.bodyA.SetAwake(true);
      this.bodyB.SetAwake(true);
      this.enableMotor = flag;
   }
   PrismaticJoint.prototype.SetMotorSpeed = function (speed) {
      if (speed === undefined) speed = 0;
      this.bodyA.SetAwake(true);
      this.bodyB.SetAwake(true);
      this.motorSpeed = speed;
   }
   PrismaticJoint.prototype.GetMotorSpeed = function () {
      return this.motorSpeed;
   }
   PrismaticJoint.prototype.SetMaxMotorForce = function (force) {
      if (force === undefined) force = 0;
      this.bodyA.SetAwake(true);
      this.bodyB.SetAwake(true);
      this.maxMotorForce = force;
   }
   PrismaticJoint.prototype.GetMotorForce = function () {
      return this.motorImpulse;
   }
   PrismaticJoint.prototype.PrismaticJoint = function (def) {
      this.__super.Joint.call(this, def);
      var tMat;
      var tX = 0;
      var tY = 0;
      this.localAnchor1.SetV(def.localAnchorA);
      this.localAnchor2.SetV(def.localAnchorB);
      this.localXAxis1.SetV(def.localAxisA);
      this.localYAxis1.x = (-this.localXAxis1.y);
      this.localYAxis1.y = this.localXAxis1.x;
      this.refAngle = def.referenceAngle;
      this.impulse.SetZero();
      this.motorMass = 0.0;
      this.motorImpulse = 0.0;
      this.lowerTranslation = def.lowerTranslation;
      this.upperTranslation = def.upperTranslation;
      this.maxMotorForce = def.maxMotorForce;
      this.motorSpeed = def.motorSpeed;
      this.enableLimit = def.enableLimit;
      this.enableMotor = def.enableMotor;
      this.limitState = Joint.e_inactiveLimit;
      this.axis.SetZero();
      this.perp.SetZero();
   }
   PrismaticJoint.prototype.InitVelocityConstraints = function (step) {
      var bA = this.bodyA;
      var bB = this.bodyB;
      var tMat;
      var tX = 0;
      this.localCenterA.SetV(bA.GetLocalCenter());
      this.localCenterB.SetV(bB.GetLocalCenter());
      var xf1 = bA.GetTransform();
      var xf2 = bB.GetTransform();
      tMat = bA.xf.R;
      var r1X = this.localAnchor1.x - this.localCenterA.x;
      var r1Y = this.localAnchor1.y - this.localCenterA.y;
      tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
      r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
      r1X = tX;
      tMat = bB.xf.R;
      var r2X = this.localAnchor2.x - this.localCenterB.x;
      var r2Y = this.localAnchor2.y - this.localCenterB.y;
      tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
      r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
      r2X = tX;
      var dX = bB.sweep.c.x + r2X - bA.sweep.c.x - r1X;
      var dY = bB.sweep.c.y + r2Y - bA.sweep.c.y - r1Y;
      this.invMassA = bA.invMass;
      this.invMassB = bB.invMass;
      this.invIA = bA.invI;
      this.invIB = bB.invI; {
         this.axis.SetV(Math.MulMV(xf1.R, this.localXAxis1));
         this.a1 = (dX + r1X) * this.axis.y - (dY + r1Y) * this.axis.x;
         this.a2 = r2X * this.axis.y - r2Y * this.axis.x;
         this.motorMass = this.invMassA + this.invMassB + this.invIA * this.a1 * this.a1 + this.invIB * this.a2 * this.a2;
         if (this.motorMass > Number.MIN_VALUE) this.motorMass = 1.0 / this.motorMass;
      } {
         this.perp.SetV(Math.MulMV(xf1.R, this.localYAxis1));
         this.s1 = (dX + r1X) * this.perp.y - (dY + r1Y) * this.perp.x;
         this.s2 = r2X * this.perp.y - r2Y * this.perp.x;
         var m1 = this.invMassA;
         var m2 = this.invMassB;
         var i1 = this.invIA;
         var i2 = this.invIB;
         this.K.col1.x = m1 + m2 + i1 * this.s1 * this.s1 + i2 * this.s2 * this.s2;
         this.K.col1.y = i1 * this.s1 + i2 * this.s2;
         this.K.col1.z = i1 * this.s1 * this.a1 + i2 * this.s2 * this.a2;
         this.K.col2.x = this.K.col1.y;
         this.K.col2.y = i1 + i2;
         this.K.col2.z = i1 * this.a1 + i2 * this.a2;
         this.K.col3.x = this.K.col1.z;
         this.K.col3.y = this.K.col2.z;
         this.K.col3.z = m1 + m2 + i1 * this.a1 * this.a1 + i2 * this.a2 * this.a2;
      }
      if (this.enableLimit) {
         var jointTransition = this.axis.x * dX + this.axis.y * dY;
         if (Math.Abs(this.upperTranslation - this.lowerTranslation) < 2.0 * Settings._linearSlop) {
            this.limitState = Joint.e_equalLimits;
         }
         else if (jointTransition <= this.lowerTranslation) {
            if (this.limitState != Joint.e_atLowerLimit) {
               this.limitState = Joint.e_atLowerLimit;
               this.impulse.z = 0.0;
            }
         }
         else if (jointTransition >= this.upperTranslation) {
            if (this.limitState != Joint.e_atUpperLimit) {
               this.limitState = Joint.e_atUpperLimit;
               this.impulse.z = 0.0;
            }
         }
         else {
            this.limitState = Joint.e_inactiveLimit;
            this.impulse.z = 0.0;
         }
      }
      else {
         this.limitState = Joint.e_inactiveLimit;
      }
      if (this.enableMotor == false) {
         this.motorImpulse = 0.0;
      }
      if (step.warmStarting) {
         this.impulse.x *= step.dtRatio;
         this.impulse.y *= step.dtRatio;
         this.motorImpulse *= step.dtRatio;
         var PX = this.impulse.x * this.perp.x + (this.motorImpulse + this.impulse.z) * this.axis.x;
         var PY = this.impulse.x * this.perp.y + (this.motorImpulse + this.impulse.z) * this.axis.y;
         var L1 = this.impulse.x * this.s1 + this.impulse.y + (this.motorImpulse + this.impulse.z) * this.a1;
         var L2 = this.impulse.x * this.s2 + this.impulse.y + (this.motorImpulse + this.impulse.z) * this.a2;
         bA.linearVelocity.x -= this.invMassA * PX;
         bA.linearVelocity.y -= this.invMassA * PY;
         bA.angularVelocity -= this.invIA * L1;
         bB.linearVelocity.x += this.invMassB * PX;
         bB.linearVelocity.y += this.invMassB * PY;
         bB.angularVelocity += this.invIB * L2;
      }
      else {
         this.impulse.SetZero();
         this.motorImpulse = 0.0;
      }
   }
   PrismaticJoint.prototype.SolveVelocityConstraints = function (step) {
      var bA = this.bodyA;
      var bB = this.bodyB;
      var v1 = bA.linearVelocity;
      var w1 = bA.angularVelocity;
      var v2 = bB.linearVelocity;
      var w2 = bB.angularVelocity;
      var PX = 0;
      var PY = 0;
      var L1 = 0;
      var L2 = 0;
      if (this.enableMotor && this.limitState != Joint.e_equalLimits) {
         var Cdot = this.axis.x * (v2.x - v1.x) + this.axis.y * (v2.y - v1.y) + this.a2 * w2 - this.a1 * w1;
         var impulse = this.motorMass * (this.motorSpeed - Cdot);
         var oldImpulse = this.motorImpulse;
         var maxImpulse = step.dt * this.maxMotorForce;
         this.motorImpulse = Math.Clamp(this.motorImpulse + impulse, (-maxImpulse), maxImpulse);
         impulse = this.motorImpulse - oldImpulse;
         PX = impulse * this.axis.x;
         PY = impulse * this.axis.y;
         L1 = impulse * this.a1;
         L2 = impulse * this.a2;
         v1.x -= this.invMassA * PX;
         v1.y -= this.invMassA * PY;
         w1 -= this.invIA * L1;
         v2.x += this.invMassB * PX;
         v2.y += this.invMassB * PY;
         w2 += this.invIB * L2;
      }
      var Cdot1X = this.perp.x * (v2.x - v1.x) + this.perp.y * (v2.y - v1.y) + this.s2 * w2 - this.s1 * w1;
      var Cdot1Y = w2 - w1;
      if (this.enableLimit && this.limitState != Joint.e_inactiveLimit) {
         var Cdot2 = this.axis.x * (v2.x - v1.x) + this.axis.y * (v2.y - v1.y) + this.a2 * w2 - this.a1 * w1;
         var f1 = this.impulse.Copy();
         var df = this.K.Solve33(new Vec3(), (-Cdot1X), (-Cdot1Y), (-Cdot2));
         this.impulse.Add(df);
         if (this.limitState == Joint.e_atLowerLimit) {
            this.impulse.z = Math.Max(this.impulse.z, 0.0);
         }
         else if (this.limitState == Joint.e_atUpperLimit) {
            this.impulse.z = Math.Min(this.impulse.z, 0.0);
         }
         var bX = (-Cdot1X) - (this.impulse.z - f1.z) * this.K.col3.x;
         var bY = (-Cdot1Y) - (this.impulse.z - f1.z) * this.K.col3.y;
         var f2r = this.K.Solve22(new Vec2(), bX, bY);
         f2r.x += f1.x;
         f2r.y += f1.y;
         this.impulse.x = f2r.x;
         this.impulse.y = f2r.y;
         df.x = this.impulse.x - f1.x;
         df.y = this.impulse.y - f1.y;
         df.z = this.impulse.z - f1.z;
         PX = df.x * this.perp.x + df.z * this.axis.x;
         PY = df.x * this.perp.y + df.z * this.axis.y;
         L1 = df.x * this.s1 + df.y + df.z * this.a1;
         L2 = df.x * this.s2 + df.y + df.z * this.a2;
         v1.x -= this.invMassA * PX;
         v1.y -= this.invMassA * PY;
         w1 -= this.invIA * L1;
         v2.x += this.invMassB * PX;
         v2.y += this.invMassB * PY;
         w2 += this.invIB * L2;
      }
      else {
         var df2 = this.K.Solve22(new Vec2(), (-Cdot1X), (-Cdot1Y));
         this.impulse.x += df2.x;
         this.impulse.y += df2.y;
         PX = df2.x * this.perp.x;
         PY = df2.x * this.perp.y;
         L1 = df2.x * this.s1 + df2.y;
         L2 = df2.x * this.s2 + df2.y;
         v1.x -= this.invMassA * PX;
         v1.y -= this.invMassA * PY;
         w1 -= this.invIA * L1;
         v2.x += this.invMassB * PX;
         v2.y += this.invMassB * PY;
         w2 += this.invIB * L2;
      }
      bA.linearVelocity.SetV(v1);
      bA.angularVelocity = w1;
      bB.linearVelocity.SetV(v2);
      bB.angularVelocity = w2;
   }
   PrismaticJoint.prototype.SolvePositionConstraints = function (baumgarte) {
      if (baumgarte === undefined) baumgarte = 0;
      var limitC = 0;
      var oldLimitImpulse = 0;
      var bA = this.bodyA;
      var bB = this.bodyB;
      var c1 = bA.sweep.c;
      var a1 = bA.sweep.a;
      var c2 = bB.sweep.c;
      var a2 = bB.sweep.a;
      var tMat;
      var tX = 0;
      var m1 = 0;
      var m2 = 0;
      var i1 = 0;
      var i2 = 0;
      var linearError = 0.0;
      var angularError = 0.0;
      var active = false;
      var C2 = 0.0;
      var R1 = Mat22.FromAngle(a1);
      var R2 = Mat22.FromAngle(a2);
      tMat = R1;
      var r1X = this.localAnchor1.x - this.localCenterA.x;
      var r1Y = this.localAnchor1.y - this.localCenterA.y;
      tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
      r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
      r1X = tX;
      tMat = R2;
      var r2X = this.localAnchor2.x - this.localCenterB.x;
      var r2Y = this.localAnchor2.y - this.localCenterB.y;
      tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
      r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
      r2X = tX;
      var dX = c2.x + r2X - c1.x - r1X;
      var dY = c2.y + r2Y - c1.y - r1Y;
      if (this.enableLimit) {
         this.axis = Math.MulMV(R1, this.localXAxis1);
         this.a1 = (dX + r1X) * this.axis.y - (dY + r1Y) * this.axis.x;
         this.a2 = r2X * this.axis.y - r2Y * this.axis.x;
         var translation = this.axis.x * dX + this.axis.y * dY;
         if (Math.Abs(this.upperTranslation - this.lowerTranslation) < 2.0 * Settings._linearSlop) {
            C2 = Math.Clamp(translation, (-Settings._maxLinearCorrection), Settings._maxLinearCorrection);
            linearError = Math.Abs(translation);
            active = true;
         }
         else if (translation <= this.lowerTranslation) {
            C2 = Math.Clamp(translation - this.lowerTranslation + Settings._linearSlop, (-Settings._maxLinearCorrection), 0.0);
            linearError = this.lowerTranslation - translation;
            active = true;
         }
         else if (translation >= this.upperTranslation) {
            C2 = Math.Clamp(translation - this.upperTranslation + Settings._linearSlop, 0.0, Settings._maxLinearCorrection);
            linearError = translation - this.upperTranslation;
            active = true;
         }
      }
      this.perp = Math.MulMV(R1, this.localYAxis1);
      this.s1 = (dX + r1X) * this.perp.y - (dY + r1Y) * this.perp.x;
      this.s2 = r2X * this.perp.y - r2Y * this.perp.x;
      var impulse = new Vec3();
      var C1X = this.perp.x * dX + this.perp.y * dY;
      var C1Y = a2 - a1 - this.refAngle;
      linearError = Math.Max(linearError, Math.Abs(C1X));
      angularError = Math.Abs(C1Y);
      if (active) {
         m1 = this.invMassA;
         m2 = this.invMassB;
         i1 = this.invIA;
         i2 = this.invIB;
         this.K.col1.x = m1 + m2 + i1 * this.s1 * this.s1 + i2 * this.s2 * this.s2;
         this.K.col1.y = i1 * this.s1 + i2 * this.s2;
         this.K.col1.z = i1 * this.s1 * this.a1 + i2 * this.s2 * this.a2;
         this.K.col2.x = this.K.col1.y;
         this.K.col2.y = i1 + i2;
         this.K.col2.z = i1 * this.a1 + i2 * this.a2;
         this.K.col3.x = this.K.col1.z;
         this.K.col3.y = this.K.col2.z;
         this.K.col3.z = m1 + m2 + i1 * this.a1 * this.a1 + i2 * this.a2 * this.a2;
         this.K.Solve33(impulse, (-C1X), (-C1Y), (-C2));
      }
      else {
         m1 = this.invMassA;
         m2 = this.invMassB;
         i1 = this.invIA;
         i2 = this.invIB;
         var k11 = m1 + m2 + i1 * this.s1 * this.s1 + i2 * this.s2 * this.s2;
         var k12 = i1 * this.s1 + i2 * this.s2;
         var k22 = i1 + i2;
         this.K.col1.Set(k11, k12, 0.0);
         this.K.col2.Set(k12, k22, 0.0);
         var impulse1 = this.K.Solve22(new Vec2(), (-C1X), (-C1Y));
         impulse.x = impulse1.x;
         impulse.y = impulse1.y;
         impulse.z = 0.0;
      }
      var PX = impulse.x * this.perp.x + impulse.z * this.axis.x;
      var PY = impulse.x * this.perp.y + impulse.z * this.axis.y;
      var L1 = impulse.x * this.s1 + impulse.y + impulse.z * this.a1;
      var L2 = impulse.x * this.s2 + impulse.y + impulse.z * this.a2;
      c1.x -= this.invMassA * PX;
      c1.y -= this.invMassA * PY;
      a1 -= this.invIA * L1;
      c2.x += this.invMassB * PX;
      c2.y += this.invMassB * PY;
      a2 += this.invIB * L2;
      bA.sweep.a = a1;
      bB.sweep.a = a2;
      bA.SynchronizeTransform();
      bB.SynchronizeTransform();
      return linearError <= Settings._linearSlop && angularError <= Settings._angularSlop;
   }
   Box2D.inherit(PrismaticJointDef, Box2D.Dynamics.Joints.JointDef);
   PrismaticJointDef.prototype.__super = Box2D.Dynamics.Joints.JointDef.prototype;
   PrismaticJointDef.PrismaticJointDef = function () {
      Box2D.Dynamics.Joints.JointDef.JointDef.apply(this, arguments);
      this.localAnchorA = new Vec2();
      this.localAnchorB = new Vec2();
      this.localAxisA = new Vec2();
   };
   PrismaticJointDef.prototype.PrismaticJointDef = function () {
      this.__super.JointDef.call(this);
      this.type = Joint.e_prismaticJoint;
      this.localAxisA.Set(1.0, 0.0);
      this.referenceAngle = 0.0;
      this.enableLimit = false;
      this.lowerTranslation = 0.0;
      this.upperTranslation = 0.0;
      this.enableMotor = false;
      this.maxMotorForce = 0.0;
      this.motorSpeed = 0.0;
   }
   PrismaticJointDef.prototype.Initialize = function (bA, bB, anchor, axis) {
      this.bodyA = bA;
      this.bodyB = bB;
      this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
      this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
      this.localAxisA = this.bodyA.GetLocalVector(axis);
      this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
   }
   Box2D.inherit(PulleyJoint, Box2D.Dynamics.Joints.Joint);
   PulleyJoint.prototype.__super = Box2D.Dynamics.Joints.Joint.prototype;
   PulleyJoint.PulleyJoint = function () {
      Box2D.Dynamics.Joints.Joint.Joint.apply(this, arguments);
      this.groundAnchor1 = new Vec2();
      this.groundAnchor2 = new Vec2();
      this.localAnchor1 = new Vec2();
      this.localAnchor2 = new Vec2();
      this.u1 = new Vec2();
      this.u2 = new Vec2();
   };
   PulleyJoint.prototype.GetAnchorA = function () {
      return this.bodyA.GetWorldPoint(this.localAnchor1);
   }
   PulleyJoint.prototype.GetAnchorB = function () {
      return this.bodyB.GetWorldPoint(this.localAnchor2);
   }
   PulleyJoint.prototype.GetReactionForce = function (inv_dt) {
      if (inv_dt === undefined) inv_dt = 0;
      return new Vec2(inv_dt * this.impulse * this.u2.x, inv_dt * this.impulse * this.u2.y);
   }
   PulleyJoint.prototype.GetReactionTorque = function (inv_dt) {
      if (inv_dt === undefined) inv_dt = 0;
      return 0.0;
   }
   PulleyJoint.prototype.GetGroundAnchorA = function () {
      var a = this.ground.xf.position.Copy();
      a.Add(this.groundAnchor1);
      return a;
   }
   PulleyJoint.prototype.GetGroundAnchorB = function () {
      var a = this.ground.xf.position.Copy();
      a.Add(this.groundAnchor2);
      return a;
   }
   PulleyJoint.prototype.GetLength1 = function () {
      var p = this.bodyA.GetWorldPoint(this.localAnchor1);
      var sX = this.ground.xf.position.x + this.groundAnchor1.x;
      var sY = this.ground.xf.position.y + this.groundAnchor1.y;
      var dX = p.x - sX;
      var dY = p.y - sY;
      return Math.sqrt(dX * dX + dY * dY);
   }
   PulleyJoint.prototype.GetLength2 = function () {
      var p = this.bodyB.GetWorldPoint(this.localAnchor2);
      var sX = this.ground.xf.position.x + this.groundAnchor2.x;
      var sY = this.ground.xf.position.y + this.groundAnchor2.y;
      var dX = p.x - sX;
      var dY = p.y - sY;
      return Math.sqrt(dX * dX + dY * dY);
   }
   PulleyJoint.prototype.GetRatio = function () {
      return this.ratio;
   }
   PulleyJoint.prototype.PulleyJoint = function (def) {
      this.__super.Joint.call(this, def);
      var tMat;
      var tX = 0;
      var tY = 0;
      this.ground = this.bodyA.world.groundBody;
      this.groundAnchor1.x = def.groundAnchorA.x - this.ground.xf.position.x;
      this.groundAnchor1.y = def.groundAnchorA.y - this.ground.xf.position.y;
      this.groundAnchor2.x = def.groundAnchorB.x - this.ground.xf.position.x;
      this.groundAnchor2.y = def.groundAnchorB.y - this.ground.xf.position.y;
      this.localAnchor1.SetV(def.localAnchorA);
      this.localAnchor2.SetV(def.localAnchorB);
      this.ratio = def.ratio;
      this.constant = def.lengthA + this.ratio * def.lengthB;
      this.maxLength1 = Math.Min(def.maxLengthA, this.constant - this.ratio * PulleyJoint._minPulleyLength);
      this.maxLength2 = Math.Min(def.maxLengthB, (this.constant - PulleyJoint._minPulleyLength) / this.ratio);
      this.impulse = 0.0;
      this.limitImpulse1 = 0.0;
      this.limitImpulse2 = 0.0;
   }
   PulleyJoint.prototype.InitVelocityConstraints = function (step) {
      var bA = this.bodyA;
      var bB = this.bodyB;
      var tMat;
      tMat = bA.xf.R;
      var r1X = this.localAnchor1.x - bA.sweep.localCenter.x;
      var r1Y = this.localAnchor1.y - bA.sweep.localCenter.y;
      var tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
      r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
      r1X = tX;
      tMat = bB.xf.R;
      var r2X = this.localAnchor2.x - bB.sweep.localCenter.x;
      var r2Y = this.localAnchor2.y - bB.sweep.localCenter.y;
      tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
      r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
      r2X = tX;
      var p1X = bA.sweep.c.x + r1X;
      var p1Y = bA.sweep.c.y + r1Y;
      var p2X = bB.sweep.c.x + r2X;
      var p2Y = bB.sweep.c.y + r2Y;
      var s1X = this.ground.xf.position.x + this.groundAnchor1.x;
      var s1Y = this.ground.xf.position.y + this.groundAnchor1.y;
      var s2X = this.ground.xf.position.x + this.groundAnchor2.x;
      var s2Y = this.ground.xf.position.y + this.groundAnchor2.y;
      this.u1.Set(p1X - s1X, p1Y - s1Y);
      this.u2.Set(p2X - s2X, p2Y - s2Y);
      var length1 = this.u1.Length();
      var length2 = this.u2.Length();
      if (length1 > Settings._linearSlop) {
         this.u1.Multiply(1.0 / length1);
      }
      else {
         this.u1.SetZero();
      }
      if (length2 > Settings._linearSlop) {
         this.u2.Multiply(1.0 / length2);
      }
      else {
         this.u2.SetZero();
      }
      var C = this.constant - length1 - this.ratio * length2;
      if (C > 0.0) {
         this.state = Joint.e_inactiveLimit;
         this.impulse = 0.0;
      }
      else {
         this.state = Joint.e_atUpperLimit;
      }
      if (length1 < this.maxLength1) {
         this.limitState1 = Joint.e_inactiveLimit;
         this.limitImpulse1 = 0.0;
      }
      else {
         this.limitState1 = Joint.e_atUpperLimit;
      }
      if (length2 < this.maxLength2) {
         this.limitState2 = Joint.e_inactiveLimit;
         this.limitImpulse2 = 0.0;
      }
      else {
         this.limitState2 = Joint.e_atUpperLimit;
      }
      var cr1u1 = r1X * this.u1.y - r1Y * this.u1.x;
      var cr2u2 = r2X * this.u2.y - r2Y * this.u2.x;
      this.limitMass1 = bA.invMass + bA.invI * cr1u1 * cr1u1;
      this.limitMass2 = bB.invMass + bB.invI * cr2u2 * cr2u2;
      this.pulleyMass = this.limitMass1 + this.ratio * this.ratio * this.limitMass2;
      this.limitMass1 = 1.0 / this.limitMass1;
      this.limitMass2 = 1.0 / this.limitMass2;
      this.pulleyMass = 1.0 / this.pulleyMass;
      if (step.warmStarting) {
         this.impulse *= step.dtRatio;
         this.limitImpulse1 *= step.dtRatio;
         this.limitImpulse2 *= step.dtRatio;
         var P1X = ((-this.impulse) - this.limitImpulse1) * this.u1.x;
         var P1Y = ((-this.impulse) - this.limitImpulse1) * this.u1.y;
         var P2X = ((-this.ratio * this.impulse) - this.limitImpulse2) * this.u2.x;
         var P2Y = ((-this.ratio * this.impulse) - this.limitImpulse2) * this.u2.y;
         bA.linearVelocity.x += bA.invMass * P1X;
         bA.linearVelocity.y += bA.invMass * P1Y;
         bA.angularVelocity += bA.invI * (r1X * P1Y - r1Y * P1X);
         bB.linearVelocity.x += bB.invMass * P2X;
         bB.linearVelocity.y += bB.invMass * P2Y;
         bB.angularVelocity += bB.invI * (r2X * P2Y - r2Y * P2X);
      }
      else {
         this.impulse = 0.0;
         this.limitImpulse1 = 0.0;
         this.limitImpulse2 = 0.0;
      }
   }
   PulleyJoint.prototype.SolveVelocityConstraints = function (step) {
      var bA = this.bodyA;
      var bB = this.bodyB;
      var tMat;
      tMat = bA.xf.R;
      var r1X = this.localAnchor1.x - bA.sweep.localCenter.x;
      var r1Y = this.localAnchor1.y - bA.sweep.localCenter.y;
      var tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
      r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
      r1X = tX;
      tMat = bB.xf.R;
      var r2X = this.localAnchor2.x - bB.sweep.localCenter.x;
      var r2Y = this.localAnchor2.y - bB.sweep.localCenter.y;
      tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
      r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
      r2X = tX;
      var v1X = 0;
      var v1Y = 0;
      var v2X = 0;
      var v2Y = 0;
      var P1X = 0;
      var P1Y = 0;
      var P2X = 0;
      var P2Y = 0;
      var Cdot = 0;
      var impulse = 0;
      var oldImpulse = 0;
      if (this.state == Joint.e_atUpperLimit) {
         v1X = bA.linearVelocity.x + ((-bA.angularVelocity * r1Y));
         v1Y = bA.linearVelocity.y + (bA.angularVelocity * r1X);
         v2X = bB.linearVelocity.x + ((-bB.angularVelocity * r2Y));
         v2Y = bB.linearVelocity.y + (bB.angularVelocity * r2X);
         Cdot = (-(this.u1.x * v1X + this.u1.y * v1Y)) - this.ratio * (this.u2.x * v2X + this.u2.y * v2Y);
         impulse = this.pulleyMass * ((-Cdot));
         oldImpulse = this.impulse;
         this.impulse = Math.Max(0.0, this.impulse + impulse);
         impulse = this.impulse - oldImpulse;
         P1X = (-impulse * this.u1.x);
         P1Y = (-impulse * this.u1.y);
         P2X = (-this.ratio * impulse * this.u2.x);
         P2Y = (-this.ratio * impulse * this.u2.y);
         bA.linearVelocity.x += bA.invMass * P1X;
         bA.linearVelocity.y += bA.invMass * P1Y;
         bA.angularVelocity += bA.invI * (r1X * P1Y - r1Y * P1X);
         bB.linearVelocity.x += bB.invMass * P2X;
         bB.linearVelocity.y += bB.invMass * P2Y;
         bB.angularVelocity += bB.invI * (r2X * P2Y - r2Y * P2X);
      }
      if (this.limitState1 == Joint.e_atUpperLimit) {
         v1X = bA.linearVelocity.x + ((-bA.angularVelocity * r1Y));
         v1Y = bA.linearVelocity.y + (bA.angularVelocity * r1X);
         Cdot = (-(this.u1.x * v1X + this.u1.y * v1Y));
         impulse = (-this.limitMass1 * Cdot);
         oldImpulse = this.limitImpulse1;
         this.limitImpulse1 = Math.Max(0.0, this.limitImpulse1 + impulse);
         impulse = this.limitImpulse1 - oldImpulse;
         P1X = (-impulse * this.u1.x);
         P1Y = (-impulse * this.u1.y);
         bA.linearVelocity.x += bA.invMass * P1X;
         bA.linearVelocity.y += bA.invMass * P1Y;
         bA.angularVelocity += bA.invI * (r1X * P1Y - r1Y * P1X);
      }
      if (this.limitState2 == Joint.e_atUpperLimit) {
         v2X = bB.linearVelocity.x + ((-bB.angularVelocity * r2Y));
         v2Y = bB.linearVelocity.y + (bB.angularVelocity * r2X);
         Cdot = (-(this.u2.x * v2X + this.u2.y * v2Y));
         impulse = (-this.limitMass2 * Cdot);
         oldImpulse = this.limitImpulse2;
         this.limitImpulse2 = Math.Max(0.0, this.limitImpulse2 + impulse);
         impulse = this.limitImpulse2 - oldImpulse;
         P2X = (-impulse * this.u2.x);
         P2Y = (-impulse * this.u2.y);
         bB.linearVelocity.x += bB.invMass * P2X;
         bB.linearVelocity.y += bB.invMass * P2Y;
         bB.angularVelocity += bB.invI * (r2X * P2Y - r2Y * P2X);
      }
   }
   PulleyJoint.prototype.SolvePositionConstraints = function (baumgarte) {
      if (baumgarte === undefined) baumgarte = 0;
      var bA = this.bodyA;
      var bB = this.bodyB;
      var tMat;
      var s1X = this.ground.xf.position.x + this.groundAnchor1.x;
      var s1Y = this.ground.xf.position.y + this.groundAnchor1.y;
      var s2X = this.ground.xf.position.x + this.groundAnchor2.x;
      var s2Y = this.ground.xf.position.y + this.groundAnchor2.y;
      var r1X = 0;
      var r1Y = 0;
      var r2X = 0;
      var r2Y = 0;
      var p1X = 0;
      var p1Y = 0;
      var p2X = 0;
      var p2Y = 0;
      var length1 = 0;
      var length2 = 0;
      var C = 0;
      var impulse = 0;
      var oldImpulse = 0;
      var oldLimitPositionImpulse = 0;
      var tX = 0;
      var linearError = 0.0;
      if (this.state == Joint.e_atUpperLimit) {
         tMat = bA.xf.R;
         r1X = this.localAnchor1.x - bA.sweep.localCenter.x;
         r1Y = this.localAnchor1.y - bA.sweep.localCenter.y;
         tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
         r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
         r1X = tX;
         tMat = bB.xf.R;
         r2X = this.localAnchor2.x - bB.sweep.localCenter.x;
         r2Y = this.localAnchor2.y - bB.sweep.localCenter.y;
         tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
         r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
         r2X = tX;
         p1X = bA.sweep.c.x + r1X;
         p1Y = bA.sweep.c.y + r1Y;
         p2X = bB.sweep.c.x + r2X;
         p2Y = bB.sweep.c.y + r2Y;
         this.u1.Set(p1X - s1X, p1Y - s1Y);
         this.u2.Set(p2X - s2X, p2Y - s2Y);
         length1 = this.u1.Length();
         length2 = this.u2.Length();
         if (length1 > Settings._linearSlop) {
            this.u1.Multiply(1.0 / length1);
         }
         else {
            this.u1.SetZero();
         }
         if (length2 > Settings._linearSlop) {
            this.u2.Multiply(1.0 / length2);
         }
         else {
            this.u2.SetZero();
         }
         C = this.constant - length1 - this.ratio * length2;
         linearError = Math.Max(linearError, (-C));
         C = Math.Clamp(C + Settings._linearSlop, (-Settings._maxLinearCorrection), 0.0);
         impulse = (-this.pulleyMass * C);
         p1X = (-impulse * this.u1.x);
         p1Y = (-impulse * this.u1.y);
         p2X = (-this.ratio * impulse * this.u2.x);
         p2Y = (-this.ratio * impulse * this.u2.y);
         bA.sweep.c.x += bA.invMass * p1X;
         bA.sweep.c.y += bA.invMass * p1Y;
         bA.sweep.a += bA.invI * (r1X * p1Y - r1Y * p1X);
         bB.sweep.c.x += bB.invMass * p2X;
         bB.sweep.c.y += bB.invMass * p2Y;
         bB.sweep.a += bB.invI * (r2X * p2Y - r2Y * p2X);
         bA.SynchronizeTransform();
         bB.SynchronizeTransform();
      }
      if (this.limitState1 == Joint.e_atUpperLimit) {
         tMat = bA.xf.R;
         r1X = this.localAnchor1.x - bA.sweep.localCenter.x;
         r1Y = this.localAnchor1.y - bA.sweep.localCenter.y;
         tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
         r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
         r1X = tX;
         p1X = bA.sweep.c.x + r1X;
         p1Y = bA.sweep.c.y + r1Y;
         this.u1.Set(p1X - s1X, p1Y - s1Y);
         length1 = this.u1.Length();
         if (length1 > Settings._linearSlop) {
            this.u1.x *= 1.0 / length1;
            this.u1.y *= 1.0 / length1;
         }
         else {
            this.u1.SetZero();
         }
         C = this.maxLength1 - length1;
         linearError = Math.Max(linearError, (-C));
         C = Math.Clamp(C + Settings._linearSlop, (-Settings._maxLinearCorrection), 0.0);
         impulse = (-this.limitMass1 * C);
         p1X = (-impulse * this.u1.x);
         p1Y = (-impulse * this.u1.y);
         bA.sweep.c.x += bA.invMass * p1X;
         bA.sweep.c.y += bA.invMass * p1Y;
         bA.sweep.a += bA.invI * (r1X * p1Y - r1Y * p1X);
         bA.SynchronizeTransform();
      }
      if (this.limitState2 == Joint.e_atUpperLimit) {
         tMat = bB.xf.R;
         r2X = this.localAnchor2.x - bB.sweep.localCenter.x;
         r2Y = this.localAnchor2.y - bB.sweep.localCenter.y;
         tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
         r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
         r2X = tX;
         p2X = bB.sweep.c.x + r2X;
         p2Y = bB.sweep.c.y + r2Y;
         this.u2.Set(p2X - s2X, p2Y - s2Y);
         length2 = this.u2.Length();
         if (length2 > Settings._linearSlop) {
            this.u2.x *= 1.0 / length2;
            this.u2.y *= 1.0 / length2;
         }
         else {
            this.u2.SetZero();
         }
         C = this.maxLength2 - length2;
         linearError = Math.Max(linearError, (-C));
         C = Math.Clamp(C + Settings._linearSlop, (-Settings._maxLinearCorrection), 0.0);
         impulse = (-this.limitMass2 * C);
         p2X = (-impulse * this.u2.x);
         p2Y = (-impulse * this.u2.y);
         bB.sweep.c.x += bB.invMass * p2X;
         bB.sweep.c.y += bB.invMass * p2Y;
         bB.sweep.a += bB.invI * (r2X * p2Y - r2Y * p2X);
         bB.SynchronizeTransform();
      }
      return linearError < Settings._linearSlop;
   }
   Box2D.postDefs.push(function () {
      Box2D.Dynamics.Joints.PulleyJoint._minPulleyLength = 2.0;
   });
   Box2D.inherit(PulleyJointDef, Box2D.Dynamics.Joints.JointDef);
   PulleyJointDef.prototype.__super = Box2D.Dynamics.Joints.JointDef.prototype;
   PulleyJointDef.PulleyJointDef = function () {
      Box2D.Dynamics.Joints.JointDef.JointDef.apply(this, arguments);
      this.groundAnchorA = new Vec2();
      this.groundAnchorB = new Vec2();
      this.localAnchorA = new Vec2();
      this.localAnchorB = new Vec2();
   };
   PulleyJointDef.prototype.PulleyJointDef = function () {
      this.__super.JointDef.call(this);
      this.type = Joint.e_pulleyJoint;
      this.groundAnchorA.Set((-1.0), 1.0);
      this.groundAnchorB.Set(1.0, 1.0);
      this.localAnchorA.Set((-1.0), 0.0);
      this.localAnchorB.Set(1.0, 0.0);
      this.lengthA = 0.0;
      this.maxLengthA = 0.0;
      this.lengthB = 0.0;
      this.maxLengthB = 0.0;
      this.ratio = 1.0;
      this.collideConnected = true;
   }
   PulleyJointDef.prototype.Initialize = function (bA, bB, gaA, gaB, anchorA, anchorB, r) {
      if (r === undefined) r = 0;
      this.bodyA = bA;
      this.bodyB = bB;
      this.groundAnchorA.SetV(gaA);
      this.groundAnchorB.SetV(gaB);
      this.localAnchorA = this.bodyA.GetLocalPoint(anchorA);
      this.localAnchorB = this.bodyB.GetLocalPoint(anchorB);
      var d1X = anchorA.x - gaA.x;
      var d1Y = anchorA.y - gaA.y;
      this.lengthA = Math.sqrt(d1X * d1X + d1Y * d1Y);
      var d2X = anchorB.x - gaB.x;
      var d2Y = anchorB.y - gaB.y;
      this.lengthB = Math.sqrt(d2X * d2X + d2Y * d2Y);
      this.ratio = r;
      var C = this.lengthA + this.ratio * this.lengthB;
      this.maxLengthA = C - this.ratio * PulleyJoint._minPulleyLength;
      this.maxLengthB = (C - PulleyJoint._minPulleyLength) / this.ratio;
   }
   Box2D.inherit(RevoluteJoint, Box2D.Dynamics.Joints.Joint);
   RevoluteJoint.prototype.__super = Box2D.Dynamics.Joints.Joint.prototype;
   RevoluteJoint.RevoluteJoint = function () {
      Box2D.Dynamics.Joints.Joint.Joint.apply(this, arguments);
      this.K = new Mat22();
      this.K1 = new Mat22();
      this.K2 = new Mat22();
      this.K3 = new Mat22();
      this.impulse3 = new Vec3();
      this.impulse2 = new Vec2();
      this.reduced = new Vec2();
      this.localAnchor1 = new Vec2();
      this.localAnchor2 = new Vec2();
      this.impulse = new Vec3();
      this.mass = new Mat33();
   };
   RevoluteJoint.prototype.GetAnchorA = function () {
      return this.bodyA.GetWorldPoint(this.localAnchor1);
   }
   RevoluteJoint.prototype.GetAnchorB = function () {
      return this.bodyB.GetWorldPoint(this.localAnchor2);
   }
   RevoluteJoint.prototype.GetReactionForce = function (inv_dt) {
      if (inv_dt === undefined) inv_dt = 0;
      return new Vec2(inv_dt * this.impulse.x, inv_dt * this.impulse.y);
   }
   RevoluteJoint.prototype.GetReactionTorque = function (inv_dt) {
      if (inv_dt === undefined) inv_dt = 0;
      return inv_dt * this.impulse.z;
   }
   RevoluteJoint.prototype.GetJointAngle = function () {
      return this.bodyB.sweep.a - this.bodyA.sweep.a - this.referenceAngle;
   }
   RevoluteJoint.prototype.GetJointSpeed = function () {
      return this.bodyB.angularVelocity - this.bodyA.angularVelocity;
   }
   RevoluteJoint.prototype.IsLimitEnabled = function () {
      return this.enableLimit;
   }
   RevoluteJoint.prototype.EnableLimit = function (flag) {
      this.enableLimit = flag;
   }
   RevoluteJoint.prototype.GetLowerLimit = function () {
      return this.lowerAngle;
   }
   RevoluteJoint.prototype.GetUpperLimit = function () {
      return this.upperAngle;
   }
   RevoluteJoint.prototype.SetLimits = function (lower, upper) {
      if (lower === undefined) lower = 0;
      if (upper === undefined) upper = 0;
      this.lowerAngle = lower;
      this.upperAngle = upper;
   }
   RevoluteJoint.prototype.IsMotorEnabled = function () {
      this.bodyA.SetAwake(true);
      this.bodyB.SetAwake(true);
      return this.enableMotor;
   }
   RevoluteJoint.prototype.EnableMotor = function (flag) {
      this.enableMotor = flag;
   }
   RevoluteJoint.prototype.SetMotorSpeed = function (speed) {
      if (speed === undefined) speed = 0;
      this.bodyA.SetAwake(true);
      this.bodyB.SetAwake(true);
      this.motorSpeed = speed;
   }
   RevoluteJoint.prototype.GetMotorSpeed = function () {
      return this.motorSpeed;
   }
   RevoluteJoint.prototype.SetMaxMotorTorque = function (torque) {
      if (torque === undefined) torque = 0;
      this.maxMotorTorque = torque;
   }
   RevoluteJoint.prototype.GetMotorTorque = function () {
      return this.maxMotorTorque;
   }
   RevoluteJoint.prototype.RevoluteJoint = function (def) {
      this.__super.Joint.call(this, def);
      this.localAnchor1.SetV(def.localAnchorA);
      this.localAnchor2.SetV(def.localAnchorB);
      this.referenceAngle = def.referenceAngle;
      this.impulse.SetZero();
      this.motorImpulse = 0.0;
      this.lowerAngle = def.lowerAngle;
      this.upperAngle = def.upperAngle;
      this.maxMotorTorque = def.maxMotorTorque;
      this.motorSpeed = def.motorSpeed;
      this.enableLimit = def.enableLimit;
      this.enableMotor = def.enableMotor;
      this.limitState = Joint.e_inactiveLimit;
   }
   RevoluteJoint.prototype.InitVelocityConstraints = function (step) {
      var bA = this.bodyA;
      var bB = this.bodyB;
      var tMat;
      var tX = 0;
      if (this.enableMotor || this.enableLimit) {}
      tMat = bA.xf.R;
      var r1X = this.localAnchor1.x - bA.sweep.localCenter.x;
      var r1Y = this.localAnchor1.y - bA.sweep.localCenter.y;
      tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
      r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
      r1X = tX;
      tMat = bB.xf.R;
      var r2X = this.localAnchor2.x - bB.sweep.localCenter.x;
      var r2Y = this.localAnchor2.y - bB.sweep.localCenter.y;
      tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
      r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
      r2X = tX;
      var m1 = bA.invMass;
      var m2 = bB.invMass;
      var i1 = bA.invI;
      var i2 = bB.invI;
      this.mass.col1.x = m1 + m2 + r1Y * r1Y * i1 + r2Y * r2Y * i2;
      this.mass.col2.x = (-r1Y * r1X * i1) - r2Y * r2X * i2;
      this.mass.col3.x = (-r1Y * i1) - r2Y * i2;
      this.mass.col1.y = this.mass.col2.x;
      this.mass.col2.y = m1 + m2 + r1X * r1X * i1 + r2X * r2X * i2;
      this.mass.col3.y = r1X * i1 + r2X * i2;
      this.mass.col1.z = this.mass.col3.x;
      this.mass.col2.z = this.mass.col3.y;
      this.mass.col3.z = i1 + i2;
      this.motorMass = 1.0 / (i1 + i2);
      if (this.enableMotor == false) {
         this.motorImpulse = 0.0;
      }
      if (this.enableLimit) {
         var jointAngle = bB.sweep.a - bA.sweep.a - this.referenceAngle;
         if (Math.Abs(this.upperAngle - this.lowerAngle) < 2.0 * Settings._angularSlop) {
            this.limitState = Joint.e_equalLimits;
         }
         else if (jointAngle <= this.lowerAngle) {
            if (this.limitState != Joint.e_atLowerLimit) {
               this.impulse.z = 0.0;
            }
            this.limitState = Joint.e_atLowerLimit;
         }
         else if (jointAngle >= this.upperAngle) {
            if (this.limitState != Joint.e_atUpperLimit) {
               this.impulse.z = 0.0;
            }
            this.limitState = Joint.e_atUpperLimit;
         }
         else {
            this.limitState = Joint.e_inactiveLimit;
            this.impulse.z = 0.0;
         }
      }
      else {
         this.limitState = Joint.e_inactiveLimit;
      }
      if (step.warmStarting) {
         this.impulse.x *= step.dtRatio;
         this.impulse.y *= step.dtRatio;
         this.motorImpulse *= step.dtRatio;
         var PX = this.impulse.x;
         var PY = this.impulse.y;
         bA.linearVelocity.x -= m1 * PX;
         bA.linearVelocity.y -= m1 * PY;
         bA.angularVelocity -= i1 * ((r1X * PY - r1Y * PX) + this.motorImpulse + this.impulse.z);
         bB.linearVelocity.x += m2 * PX;
         bB.linearVelocity.y += m2 * PY;
         bB.angularVelocity += i2 * ((r2X * PY - r2Y * PX) + this.motorImpulse + this.impulse.z);
      }
      else {
         this.impulse.SetZero();
         this.motorImpulse = 0.0;
      }
   }
   RevoluteJoint.prototype.SolveVelocityConstraints = function (step) {
      var bA = this.bodyA;
      var bB = this.bodyB;
      var tMat;
      var tX = 0;
      var newImpulse = 0;
      var r1X = 0;
      var r1Y = 0;
      var r2X = 0;
      var r2Y = 0;
      var v1 = bA.linearVelocity;
      var w1 = bA.angularVelocity;
      var v2 = bB.linearVelocity;
      var w2 = bB.angularVelocity;
      var m1 = bA.invMass;
      var m2 = bB.invMass;
      var i1 = bA.invI;
      var i2 = bB.invI;
      if (this.enableMotor && this.limitState != Joint.e_equalLimits) {
         var Cdot = w2 - w1 - this.motorSpeed;
         var impulse = this.motorMass * ((-Cdot));
         var oldImpulse = this.motorImpulse;
         var maxImpulse = step.dt * this.maxMotorTorque;
         this.motorImpulse = Math.Clamp(this.motorImpulse + impulse, (-maxImpulse), maxImpulse);
         impulse = this.motorImpulse - oldImpulse;
         w1 -= i1 * impulse;
         w2 += i2 * impulse;
      }
      if (this.enableLimit && this.limitState != Joint.e_inactiveLimit) {
         tMat = bA.xf.R;
         r1X = this.localAnchor1.x - bA.sweep.localCenter.x;
         r1Y = this.localAnchor1.y - bA.sweep.localCenter.y;
         tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
         r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
         r1X = tX;
         tMat = bB.xf.R;
         r2X = this.localAnchor2.x - bB.sweep.localCenter.x;
         r2Y = this.localAnchor2.y - bB.sweep.localCenter.y;
         tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
         r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
         r2X = tX;
         var Cdot1X = v2.x + ((-w2 * r2Y)) - v1.x - ((-w1 * r1Y));
         var Cdot1Y = v2.y + (w2 * r2X) - v1.y - (w1 * r1X);
         var Cdot2 = w2 - w1;
         this.mass.Solve33(this.impulse3, (-Cdot1X), (-Cdot1Y), (-Cdot2));
         if (this.limitState == Joint.e_equalLimits) {
            this.impulse.Add(this.impulse3);
         }
         else if (this.limitState == Joint.e_atLowerLimit) {
            newImpulse = this.impulse.z + this.impulse3.z;
            if (newImpulse < 0.0) {
               this.mass.Solve22(this.reduced, (-Cdot1X), (-Cdot1Y));
               this.impulse3.x = this.reduced.x;
               this.impulse3.y = this.reduced.y;
               this.impulse3.z = (-this.impulse.z);
               this.impulse.x += this.reduced.x;
               this.impulse.y += this.reduced.y;
               this.impulse.z = 0.0;
            }
         }
         else if (this.limitState == Joint.e_atUpperLimit) {
            newImpulse = this.impulse.z + this.impulse3.z;
            if (newImpulse > 0.0) {
               this.mass.Solve22(this.reduced, (-Cdot1X), (-Cdot1Y));
               this.impulse3.x = this.reduced.x;
               this.impulse3.y = this.reduced.y;
               this.impulse3.z = (-this.impulse.z);
               this.impulse.x += this.reduced.x;
               this.impulse.y += this.reduced.y;
               this.impulse.z = 0.0;
            }
         }
         v1.x -= m1 * this.impulse3.x;
         v1.y -= m1 * this.impulse3.y;
         w1 -= i1 * (r1X * this.impulse3.y - r1Y * this.impulse3.x + this.impulse3.z);
         v2.x += m2 * this.impulse3.x;
         v2.y += m2 * this.impulse3.y;
         w2 += i2 * (r2X * this.impulse3.y - r2Y * this.impulse3.x + this.impulse3.z);
      }
      else {
         tMat = bA.xf.R;
         r1X = this.localAnchor1.x - bA.sweep.localCenter.x;
         r1Y = this.localAnchor1.y - bA.sweep.localCenter.y;
         tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
         r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
         r1X = tX;
         tMat = bB.xf.R;
         r2X = this.localAnchor2.x - bB.sweep.localCenter.x;
         r2Y = this.localAnchor2.y - bB.sweep.localCenter.y;
         tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
         r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
         r2X = tX;
         var CdotX = v2.x + ((-w2 * r2Y)) - v1.x - ((-w1 * r1Y));
         var CdotY = v2.y + (w2 * r2X) - v1.y - (w1 * r1X);
         this.mass.Solve22(this.impulse2, (-CdotX), (-CdotY));
         this.impulse.x += this.impulse2.x;
         this.impulse.y += this.impulse2.y;
         v1.x -= m1 * this.impulse2.x;
         v1.y -= m1 * this.impulse2.y;
         w1 -= i1 * (r1X * this.impulse2.y - r1Y * this.impulse2.x);
         v2.x += m2 * this.impulse2.x;
         v2.y += m2 * this.impulse2.y;
         w2 += i2 * (r2X * this.impulse2.y - r2Y * this.impulse2.x);
      }
      bA.linearVelocity.SetV(v1);
      bA.angularVelocity = w1;
      bB.linearVelocity.SetV(v2);
      bB.angularVelocity = w2;
   }
   RevoluteJoint.prototype.SolvePositionConstraints = function (baumgarte) {
      if (baumgarte === undefined) baumgarte = 0;
      var oldLimitImpulse = 0;
      var C = 0;
      var tMat;
      var bA = this.bodyA;
      var bB = this.bodyB;
      var angularError = 0.0;
      var positionError = 0.0;
      var tX = 0;
      var impulseX = 0;
      var impulseY = 0;
      if (this.enableLimit && this.limitState != Joint.e_inactiveLimit) {
         var angle = bB.sweep.a - bA.sweep.a - this.referenceAngle;
         var limitImpulse = 0.0;
         if (this.limitState == Joint.e_equalLimits) {
            C = Math.Clamp(angle - this.lowerAngle, (-Settings._maxAngularCorrection), Settings._maxAngularCorrection);
            limitImpulse = (-this.motorMass * C);
            angularError = Math.Abs(C);
         }
         else if (this.limitState == Joint.e_atLowerLimit) {
            C = angle - this.lowerAngle;
            angularError = (-C);
            C = Math.Clamp(C + Settings._angularSlop, (-Settings._maxAngularCorrection), 0.0);
            limitImpulse = (-this.motorMass * C);
         }
         else if (this.limitState == Joint.e_atUpperLimit) {
            C = angle - this.upperAngle;
            angularError = C;
            C = Math.Clamp(C - Settings._angularSlop, 0.0, Settings._maxAngularCorrection);
            limitImpulse = (-this.motorMass * C);
         }
         bA.sweep.a -= bA.invI * limitImpulse;
         bB.sweep.a += bB.invI * limitImpulse;
         bA.SynchronizeTransform();
         bB.SynchronizeTransform();
      } {
         tMat = bA.xf.R;
         var r1X = this.localAnchor1.x - bA.sweep.localCenter.x;
         var r1Y = this.localAnchor1.y - bA.sweep.localCenter.y;
         tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
         r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
         r1X = tX;
         tMat = bB.xf.R;
         var r2X = this.localAnchor2.x - bB.sweep.localCenter.x;
         var r2Y = this.localAnchor2.y - bB.sweep.localCenter.y;
         tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
         r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
         r2X = tX;
         var CX = bB.sweep.c.x + r2X - bA.sweep.c.x - r1X;
         var CY = bB.sweep.c.y + r2Y - bA.sweep.c.y - r1Y;
         var CLengthSquared = CX * CX + CY * CY;
         var CLength = Math.sqrt(CLengthSquared);
         positionError = CLength;
         var invMass1 = bA.invMass;
         var invMass2 = bB.invMass;
         var invI1 = bA.invI;
         var invI2 = bB.invI;
         var k_allowedStretch = 10.0 * Settings._linearSlop;
         if (CLengthSquared > k_allowedStretch * k_allowedStretch) {
            var uX = CX / CLength;
            var uY = CY / CLength;
            var k = invMass1 + invMass2;
            var m = 1.0 / k;
            impulseX = m * ((-CX));
            impulseY = m * ((-CY));
            var k_beta = 0.5;
            bA.sweep.c.x -= k_beta * invMass1 * impulseX;
            bA.sweep.c.y -= k_beta * invMass1 * impulseY;
            bB.sweep.c.x += k_beta * invMass2 * impulseX;
            bB.sweep.c.y += k_beta * invMass2 * impulseY;
            CX = bB.sweep.c.x + r2X - bA.sweep.c.x - r1X;
            CY = bB.sweep.c.y + r2Y - bA.sweep.c.y - r1Y;
         }
         this.K1.col1.x = invMass1 + invMass2;
         this.K1.col2.x = 0.0;
         this.K1.col1.y = 0.0;
         this.K1.col2.y = invMass1 + invMass2;
         this.K2.col1.x = invI1 * r1Y * r1Y;
         this.K2.col2.x = (-invI1 * r1X * r1Y);
         this.K2.col1.y = (-invI1 * r1X * r1Y);
         this.K2.col2.y = invI1 * r1X * r1X;
         this.K3.col1.x = invI2 * r2Y * r2Y;
         this.K3.col2.x = (-invI2 * r2X * r2Y);
         this.K3.col1.y = (-invI2 * r2X * r2Y);
         this.K3.col2.y = invI2 * r2X * r2X;
         this.K.SetM(this.K1);
         this.K.AddM(this.K2);
         this.K.AddM(this.K3);
         this.K.Solve(RevoluteJoint.tImpulse, (-CX), (-CY));
         impulseX = RevoluteJoint.tImpulse.x;
         impulseY = RevoluteJoint.tImpulse.y;
         bA.sweep.c.x -= bA.invMass * impulseX;
         bA.sweep.c.y -= bA.invMass * impulseY;
         bA.sweep.a -= bA.invI * (r1X * impulseY - r1Y * impulseX);
         bB.sweep.c.x += bB.invMass * impulseX;
         bB.sweep.c.y += bB.invMass * impulseY;
         bB.sweep.a += bB.invI * (r2X * impulseY - r2Y * impulseX);
         bA.SynchronizeTransform();
         bB.SynchronizeTransform();
      }
      return positionError <= Settings._linearSlop && angularError <= Settings._angularSlop;
   }
   Box2D.postDefs.push(function () {
      Box2D.Dynamics.Joints.RevoluteJoint.tImpulse = new Vec2();
   });
   Box2D.inherit(RevoluteJointDef, Box2D.Dynamics.Joints.JointDef);
   RevoluteJointDef.prototype.__super = Box2D.Dynamics.Joints.JointDef.prototype;
   RevoluteJointDef.RevoluteJointDef = function () {
      Box2D.Dynamics.Joints.JointDef.JointDef.apply(this, arguments);
      this.localAnchorA = new Vec2();
      this.localAnchorB = new Vec2();
   };
   RevoluteJointDef.prototype.RevoluteJointDef = function () {
      this.__super.JointDef.call(this);
      this.type = Joint.e_revoluteJoint;
      this.localAnchorA.Set(0.0, 0.0);
      this.localAnchorB.Set(0.0, 0.0);
      this.referenceAngle = 0.0;
      this.lowerAngle = 0.0;
      this.upperAngle = 0.0;
      this.maxMotorTorque = 0.0;
      this.motorSpeed = 0.0;
      this.enableLimit = false;
      this.enableMotor = false;
   }
   RevoluteJointDef.prototype.Initialize = function (bA, bB, anchor) {
      this.bodyA = bA;
      this.bodyB = bB;
      this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
      this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
      this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
   }
   Box2D.inherit(WeldJoint, Box2D.Dynamics.Joints.Joint);
   WeldJoint.prototype.__super = Box2D.Dynamics.Joints.Joint.prototype;
   WeldJoint.WeldJoint = function () {
      Box2D.Dynamics.Joints.Joint.Joint.apply(this, arguments);
      this.localAnchorA = new Vec2();
      this.localAnchorB = new Vec2();
      this.impulse = new Vec3();
      this.mass = new Mat33();
   };
   WeldJoint.prototype.GetAnchorA = function () {
      return this.bodyA.GetWorldPoint(this.localAnchorA);
   }
   WeldJoint.prototype.GetAnchorB = function () {
      return this.bodyB.GetWorldPoint(this.localAnchorB);
   }
   WeldJoint.prototype.GetReactionForce = function (inv_dt) {
      if (inv_dt === undefined) inv_dt = 0;
      return new Vec2(inv_dt * this.impulse.x, inv_dt * this.impulse.y);
   }
   WeldJoint.prototype.GetReactionTorque = function (inv_dt) {
      if (inv_dt === undefined) inv_dt = 0;
      return inv_dt * this.impulse.z;
   }
   WeldJoint.prototype.WeldJoint = function (def) {
      this.__super.Joint.call(this, def);
      this.localAnchorA.SetV(def.localAnchorA);
      this.localAnchorB.SetV(def.localAnchorB);
      this.referenceAngle = def.referenceAngle;
      this.impulse.SetZero();
      this.mass = new Mat33();
   }
   WeldJoint.prototype.InitVelocityConstraints = function (step) {
      var tMat;
      var tX = 0;
      var bA = this.bodyA;
      var bB = this.bodyB;
      tMat = bA.xf.R;
      var rAX = this.localAnchorA.x - bA.sweep.localCenter.x;
      var rAY = this.localAnchorA.y - bA.sweep.localCenter.y;
      tX = (tMat.col1.x * rAX + tMat.col2.x * rAY);
      rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY);
      rAX = tX;
      tMat = bB.xf.R;
      var rBX = this.localAnchorB.x - bB.sweep.localCenter.x;
      var rBY = this.localAnchorB.y - bB.sweep.localCenter.y;
      tX = (tMat.col1.x * rBX + tMat.col2.x * rBY);
      rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY);
      rBX = tX;
      var mA = bA.invMass;
      var mB = bB.invMass;
      var iA = bA.invI;
      var iB = bB.invI;
      this.mass.col1.x = mA + mB + rAY * rAY * iA + rBY * rBY * iB;
      this.mass.col2.x = (-rAY * rAX * iA) - rBY * rBX * iB;
      this.mass.col3.x = (-rAY * iA) - rBY * iB;
      this.mass.col1.y = this.mass.col2.x;
      this.mass.col2.y = mA + mB + rAX * rAX * iA + rBX * rBX * iB;
      this.mass.col3.y = rAX * iA + rBX * iB;
      this.mass.col1.z = this.mass.col3.x;
      this.mass.col2.z = this.mass.col3.y;
      this.mass.col3.z = iA + iB;
      if (step.warmStarting) {
         this.impulse.x *= step.dtRatio;
         this.impulse.y *= step.dtRatio;
         this.impulse.z *= step.dtRatio;
         bA.linearVelocity.x -= mA * this.impulse.x;
         bA.linearVelocity.y -= mA * this.impulse.y;
         bA.angularVelocity -= iA * (rAX * this.impulse.y - rAY * this.impulse.x + this.impulse.z);
         bB.linearVelocity.x += mB * this.impulse.x;
         bB.linearVelocity.y += mB * this.impulse.y;
         bB.angularVelocity += iB * (rBX * this.impulse.y - rBY * this.impulse.x + this.impulse.z);
      }
      else {
         this.impulse.SetZero();
      }
   }
   WeldJoint.prototype.SolveVelocityConstraints = function (step) {
      var tMat;
      var tX = 0;
      var bA = this.bodyA;
      var bB = this.bodyB;
      var vA = bA.linearVelocity;
      var wA = bA.angularVelocity;
      var vB = bB.linearVelocity;
      var wB = bB.angularVelocity;
      var mA = bA.invMass;
      var mB = bB.invMass;
      var iA = bA.invI;
      var iB = bB.invI;
      tMat = bA.xf.R;
      var rAX = this.localAnchorA.x - bA.sweep.localCenter.x;
      var rAY = this.localAnchorA.y - bA.sweep.localCenter.y;
      tX = (tMat.col1.x * rAX + tMat.col2.x * rAY);
      rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY);
      rAX = tX;
      tMat = bB.xf.R;
      var rBX = this.localAnchorB.x - bB.sweep.localCenter.x;
      var rBY = this.localAnchorB.y - bB.sweep.localCenter.y;
      tX = (tMat.col1.x * rBX + tMat.col2.x * rBY);
      rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY);
      rBX = tX;
      var Cdot1X = vB.x - wB * rBY - vA.x + wA * rAY;
      var Cdot1Y = vB.y + wB * rBX - vA.y - wA * rAX;
      var Cdot2 = wB - wA;
      var impulse = new Vec3();
      this.mass.Solve33(impulse, (-Cdot1X), (-Cdot1Y), (-Cdot2));
      this.impulse.Add(impulse);
      vA.x -= mA * impulse.x;
      vA.y -= mA * impulse.y;
      wA -= iA * (rAX * impulse.y - rAY * impulse.x + impulse.z);
      vB.x += mB * impulse.x;
      vB.y += mB * impulse.y;
      wB += iB * (rBX * impulse.y - rBY * impulse.x + impulse.z);
      bA.angularVelocity = wA;
      bB.angularVelocity = wB;
   }
   WeldJoint.prototype.SolvePositionConstraints = function (baumgarte) {
      if (baumgarte === undefined) baumgarte = 0;
      var tMat;
      var tX = 0;
      var bA = this.bodyA;
      var bB = this.bodyB;
      tMat = bA.xf.R;
      var rAX = this.localAnchorA.x - bA.sweep.localCenter.x;
      var rAY = this.localAnchorA.y - bA.sweep.localCenter.y;
      tX = (tMat.col1.x * rAX + tMat.col2.x * rAY);
      rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY);
      rAX = tX;
      tMat = bB.xf.R;
      var rBX = this.localAnchorB.x - bB.sweep.localCenter.x;
      var rBY = this.localAnchorB.y - bB.sweep.localCenter.y;
      tX = (tMat.col1.x * rBX + tMat.col2.x * rBY);
      rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY);
      rBX = tX;
      var mA = bA.invMass;
      var mB = bB.invMass;
      var iA = bA.invI;
      var iB = bB.invI;
      var C1X = bB.sweep.c.x + rBX - bA.sweep.c.x - rAX;
      var C1Y = bB.sweep.c.y + rBY - bA.sweep.c.y - rAY;
      var C2 = bB.sweep.a - bA.sweep.a - this.referenceAngle;
      var k_allowedStretch = 10.0 * Settings._linearSlop;
      var positionError = Math.sqrt(C1X * C1X + C1Y * C1Y);
      var angularError = Math.Abs(C2);
      if (positionError > k_allowedStretch) {
         iA *= 1.0;
         iB *= 1.0;
      }
      this.mass.col1.x = mA + mB + rAY * rAY * iA + rBY * rBY * iB;
      this.mass.col2.x = (-rAY * rAX * iA) - rBY * rBX * iB;
      this.mass.col3.x = (-rAY * iA) - rBY * iB;
      this.mass.col1.y = this.mass.col2.x;
      this.mass.col2.y = mA + mB + rAX * rAX * iA + rBX * rBX * iB;
      this.mass.col3.y = rAX * iA + rBX * iB;
      this.mass.col1.z = this.mass.col3.x;
      this.mass.col2.z = this.mass.col3.y;
      this.mass.col3.z = iA + iB;
      var impulse = new Vec3();
      this.mass.Solve33(impulse, (-C1X), (-C1Y), (-C2));
      bA.sweep.c.x -= mA * impulse.x;
      bA.sweep.c.y -= mA * impulse.y;
      bA.sweep.a -= iA * (rAX * impulse.y - rAY * impulse.x + impulse.z);
      bB.sweep.c.x += mB * impulse.x;
      bB.sweep.c.y += mB * impulse.y;
      bB.sweep.a += iB * (rBX * impulse.y - rBY * impulse.x + impulse.z);
      bA.SynchronizeTransform();
      bB.SynchronizeTransform();
      return positionError <= Settings._linearSlop && angularError <= Settings._angularSlop;
   }
   Box2D.inherit(WeldJointDef, Box2D.Dynamics.Joints.JointDef);
   WeldJointDef.prototype.__super = Box2D.Dynamics.Joints.JointDef.prototype;
   WeldJointDef.WeldJointDef = function () {
      Box2D.Dynamics.Joints.JointDef.JointDef.apply(this, arguments);
      this.localAnchorA = new Vec2();
      this.localAnchorB = new Vec2();
   };
   WeldJointDef.prototype.WeldJointDef = function () {
      this.__super.JointDef.call(this);
      this.type = Joint.e_weldJoint;
      this.referenceAngle = 0.0;
   }
   WeldJointDef.prototype.Initialize = function (bA, bB, anchor) {
      this.bodyA = bA;
      this.bodyB = bB;
      this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchor));
      this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchor));
      this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
   }
})();
(function () {
   var DebugDraw = Box2D.Dynamics.DebugDraw;
   DebugDraw.DebugDraw = function () {
      this.drawScale = 1.0;
      this.lineThickness = 1.0;
      this.alpha = 1.0;
      this.fillAlpha = 1.0;
      this.xformScale = 1.0;
      var __this = this;
      //#WORKAROUND
      this.sprite = {
         graphics: {
            clear: function () {
               __this.ctx.clearRect(0, 0, __this.ctx.canvas.width, __this.ctx.canvas.height)
            }
         }
      };
   };
   DebugDraw.prototype._color = function (color, alpha) {
      return "rgba(" + ((color & 0xFF0000) >> 16) + "," + ((color & 0xFF00) >> 8) + "," + (color & 0xFF) + "," + alpha + ")";
   };
   DebugDraw.prototype.DebugDraw = function () {
      this.drawFlags = 0;
   };
   DebugDraw.prototype.SetFlags = function (flags) {
      if (flags === undefined) flags = 0;
      this.drawFlags = flags;
   };
   DebugDraw.prototype.GetFlags = function () {
      return this.drawFlags;
   };
   DebugDraw.prototype.AppendFlags = function (flags) {
      if (flags === undefined) flags = 0;
      this.drawFlags |= flags;
   };
   DebugDraw.prototype.ClearFlags = function (flags) {
      if (flags === undefined) flags = 0;
      this.drawFlags &= ~flags;
   };
   DebugDraw.prototype.SetSprite = function (sprite) {
      this.ctx = sprite;
   };
   DebugDraw.prototype.GetSprite = function () {
      return this.ctx;
   };
   DebugDraw.prototype.SetDrawScale = function (drawScale) {
      if (drawScale === undefined) drawScale = 0;
      this.drawScale = drawScale;
   };
   DebugDraw.prototype.GetDrawScale = function () {
      return this.drawScale;
   };
   DebugDraw.prototype.SetLineThickness = function (lineThickness) {
      if (lineThickness === undefined) lineThickness = 0;
      this.lineThickness = lineThickness;
      this.ctx.strokeWidth = lineThickness;
   };
   DebugDraw.prototype.GetLineThickness = function () {
      return this.lineThickness;
   };
   DebugDraw.prototype.SetAlpha = function (alpha) {
      if (alpha === undefined) alpha = 0;
      this.alpha = alpha;
   };
   DebugDraw.prototype.GetAlpha = function () {
      return this.alpha;
   };
   DebugDraw.prototype.SetFillAlpha = function (alpha) {
      if (alpha === undefined) alpha = 0;
      this.fillAlpha = alpha;
   };
   DebugDraw.prototype.GetFillAlpha = function () {
      return this.fillAlpha;
   };
   DebugDraw.prototype.SetXFormScale = function (xformScale) {
      if (xformScale === undefined) xformScale = 0;
      this.xformScale = xformScale;
   };
   DebugDraw.prototype.GetXFormScale = function () {
      return this.xformScale;
   };
   DebugDraw.prototype.DrawPolygon = function (vertices, vertexCount, color) {
      if (!vertexCount) return;
      var s = this.ctx;
      var drawScale = this.drawScale;
      s.beginPath();
      s.strokeStyle = this._color(color.color, this.alpha);
      s.moveTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
      for (var i = 1; i < vertexCount; i++) {
         s.lineTo(vertices[i].x * drawScale, vertices[i].y * drawScale);
      }
      s.lineTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
      s.closePath();
      s.stroke();
   };
   DebugDraw.prototype.DrawSolidPolygon = function (vertices, vertexCount, color) {
      if (!vertexCount) return;
      var s = this.ctx;
      var drawScale = this.drawScale;
      s.beginPath();
      s.strokeStyle = this._color(color.color, this.alpha);
      s.fillStyle = this._color(color.color, this.fillAlpha);
      s.moveTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
      for (var i = 1; i < vertexCount; i++) {
         s.lineTo(vertices[i].x * drawScale, vertices[i].y * drawScale);
      }
      s.lineTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
      s.closePath();
      s.fill();
      s.stroke();
   };
   DebugDraw.prototype.DrawCircle = function (center, radius, color) {
      if (!radius) return;
      var s = this.ctx;
      var drawScale = this.drawScale;
      s.beginPath();
      s.strokeStyle = this._color(color.color, this.alpha);
      s.arc(center.x * drawScale, center.y * drawScale, radius * drawScale, 0, Math.PI * 2, true);
      s.closePath();
      s.stroke();
   };
   DebugDraw.prototype.DrawSolidCircle = function (center, radius, axis, color) {
      if (!radius) return;
      var s = this.ctx,
         drawScale = this.drawScale,
         cx = center.x * drawScale,
         cy = center.y * drawScale;
      s.moveTo(0, 0);
      s.beginPath();
      s.strokeStyle = this._color(color.color, this.alpha);
      s.fillStyle = this._color(color.color, this.fillAlpha);
      s.arc(cx, cy, radius * drawScale, 0, Math.PI * 2, true);
      s.moveTo(cx, cy);
      s.lineTo((center.x + axis.x * radius) * drawScale, (center.y + axis.y * radius) * drawScale);
      s.closePath();
      s.fill();
      s.stroke();
   };
   DebugDraw.prototype.DrawSegment = function (p1, p2, color) {
      var s = this.ctx,
         drawScale = this.drawScale;
      s.strokeStyle = this._color(color.color, this.alpha);
      s.beginPath();
      s.moveTo(p1.x * drawScale, p1.y * drawScale);
      s.lineTo(p2.x * drawScale, p2.y * drawScale);
      s.closePath();
      s.stroke();
   };
   DebugDraw.prototype.DrawTransform = function (xf) {
      var s = this.ctx,
         drawScale = this.drawScale;
      s.beginPath();
      s.strokeStyle = this._color(0xff0000, this.alpha);
      s.moveTo(xf.position.x * drawScale, xf.position.y * drawScale);
      s.lineTo((xf.position.x + this.xformScale * xf.R.col1.x) * drawScale, (xf.position.y + this.xformScale * xf.R.col1.y) * drawScale);

      s.strokeStyle = this._color(0xff00, this.alpha);
      s.moveTo(xf.position.x * drawScale, xf.position.y * drawScale);
      s.lineTo((xf.position.x + this.xformScale * xf.R.col2.x) * drawScale, (xf.position.y + this.xformScale * xf.R.col2.y) * drawScale);
      s.closePath();
      s.stroke();
   };
})(); //post-definitions
var i;
for (i = 0; i < Box2D.postDefs.length; ++i) Box2D.postDefs[i]();
delete Box2D.postDefs;
