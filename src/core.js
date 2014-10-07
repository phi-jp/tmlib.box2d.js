/*
 * tm.box2d
 */




(function() {
var b2Color = Box2D.Common.b2Color,
  b2internal = Box2D.Common.b2internal,
  b2Settings = Box2D.Common.b2Settings,
  b2CircleShape = Box2D.Collision.Shapes.b2CircleShape,
  b2EdgeChainDef = Box2D.Collision.Shapes.b2EdgeChainDef,
  b2EdgeShape = Box2D.Collision.Shapes.b2EdgeShape,
  b2MassData = Box2D.Collision.Shapes.b2MassData,
  b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape,
  b2Shape = Box2D.Collision.Shapes.b2Shape,
  b2Mat22 = Box2D.Common.Math.b2Mat22,
  b2Mat33 = Box2D.Common.Math.b2Mat33,
  b2Math = Box2D.Common.Math.b2Math,
  b2Sweep = Box2D.Common.Math.b2Sweep,
  b2Transform = Box2D.Common.Math.b2Transform,
  b2Vec2 = Box2D.Common.Math.b2Vec2,
  b2Vec3 = Box2D.Common.Math.b2Vec3,
  b2Body = Box2D.Dynamics.b2Body,
  b2BodyDef = Box2D.Dynamics.b2BodyDef,
  b2ContactFilter = Box2D.Dynamics.b2ContactFilter,
  b2ContactImpulse = Box2D.Dynamics.b2ContactImpulse,
  b2ContactListener = Box2D.Dynamics.b2ContactListener,
  b2ContactManager = Box2D.Dynamics.b2ContactManager,
  b2DebugDraw = Box2D.Dynamics.b2DebugDraw,
  b2DestructionListener = Box2D.Dynamics.b2DestructionListener,
  b2FilterData = Box2D.Dynamics.b2FilterData,
  b2Fixture = Box2D.Dynamics.b2Fixture,
  b2FixtureDef = Box2D.Dynamics.b2FixtureDef,
  b2Island = Box2D.Dynamics.b2Island,
  b2TimeStep = Box2D.Dynamics.b2TimeStep,
  b2World = Box2D.Dynamics.b2World,
  b2AABB = Box2D.Collision.b2AABB,
  b2Bound = Box2D.Collision.b2Bound,
  b2BoundValues = Box2D.Collision.b2BoundValues,
  b2Collision = Box2D.Collision.b2Collision,
  b2ContactID = Box2D.Collision.b2ContactID,
  b2ContactPoint = Box2D.Collision.b2ContactPoint,
  b2Distance = Box2D.Collision.b2Distance,
  b2DistanceInput = Box2D.Collision.b2DistanceInput,
  b2DistanceOutput = Box2D.Collision.b2DistanceOutput,
  b2DistanceProxy = Box2D.Collision.b2DistanceProxy,
  b2DynamicTree = Box2D.Collision.b2DynamicTree,
  b2DynamicTreeBroadPhase = Box2D.Collision.b2DynamicTreeBroadPhase,
  b2DynamicTreeNode = Box2D.Collision.b2DynamicTreeNode,
  b2DynamicTreePair = Box2D.Collision.b2DynamicTreePair,
  b2Manifold = Box2D.Collision.b2Manifold,
  b2ManifoldPoint = Box2D.Collision.b2ManifoldPoint,
  b2Point = Box2D.Collision.b2Point,
  b2RayCastInput = Box2D.Collision.b2RayCastInput,
  b2RayCastOutput = Box2D.Collision.b2RayCastOutput,
  b2Segment = Box2D.Collision.b2Segment,
  b2SeparationFunction = Box2D.Collision.b2SeparationFunction,
  b2Simplex = Box2D.Collision.b2Simplex,
  b2SimplexCache = Box2D.Collision.b2SimplexCache,
  b2SimplexVertex = Box2D.Collision.b2SimplexVertex,
  b2TimeOfImpact = Box2D.Collision.b2TimeOfImpact,
  b2TOIInput = Box2D.Collision.b2TOIInput,
  b2WorldManifold = Box2D.Collision.b2WorldManifold,
  ClipVertex = Box2D.Collision.ClipVertex,
  Features = Box2D.Collision.Features,
  IBroadPhase = Box2D.Collision.IBroadPhase;


var WORLD_SCALE     = 46;   // 1 meter = 46pixels
var WORLD_SIZE = 640;
var WORLD_HARF = Math.floor(WORLD_SIZE / 2);

    tm.define("tm.box2d.Scene", {
        superClass: "tm.app.Scene",

        init: function(param) {
            this.superInit();

            var gravity = new b2Vec2(0, 9.8);
            this.world = new b2World(gravity, true);

            var debugCanvas = tm.graphics.Canvas();
            debugCanvas.width  = param.width;
            debugCanvas.height = param.height;
            var debugSprite = tm.app.Sprite(debugCanvas, SCREEN_WIDTH, SCREEN_HEIGHT);
            debugSprite.origin.set(0, 0);
            debugSprite.x = debugSprite.y = 0;
            this.addChild(debugSprite);

            var debugDraw = new b2DebugDraw();
            debugDraw.SetSprite(debugCanvas.context);
            debugDraw.SetDrawScale(WORLD_SCALE);
            debugDraw.SetLineThickness(1.0);
            debugDraw.SetAlpha(1);
            debugDraw.SetFillAlpha(0.4);
            debugDraw.SetFlags(b2DebugDraw.e_shapeBit);
            this.world.SetDebugDraw(debugDraw);

            // 床の生成
            var groundBodyDef = new b2BodyDef();
            groundBodyDef.type = b2Body.b2_staticBody;
            groundBodyDef.position.Set(WORLD_HARF / WORLD_SCALE
                                    , (WORLD_SIZE - 10) / WORLD_SCALE);
            var groundBody = this.world.CreateBody(groundBodyDef);
            
            var groundBox = new b2PolygonShape();
            groundBox.SetAsBox(WORLD_SIZE / 2 / WORLD_SCALE, 10 / 2 / WORLD_SCALE);
            
            var groundFixtureDef = new b2FixtureDef();
            groundFixtureDef.shape = groundBox;
            groundFixtureDef.density = 1;
            groundFixtureDef.friction = 1;
            groundBody.CreateFixture(groundFixtureDef);
        },

        update: function(app) {
            var velocityIterations = 1;
            var positionIterations = 1;
            // 物理空間の更新
            this.world.Step(1/app.fps,velocityIterations,positionIterations);
            // debug画面の更新
            this.world.ClearForces();
            this.world.DrawDebugData();
        },

        onpointingstart: function(app) {
            var p = app.pointing;
            this.createBall(p.x, p.y);
        },

        createBall: function(x, y) {
            var bodyDef = new b2BodyDef();
            bodyDef.type = b2Body.b2_dynamicBody;
            // タッチした位置に生成
            bodyDef.position.Set(x/WORLD_SCALE, y/WORLD_SCALE);

            var circleFix = new b2FixtureDef();
            var circlePly = new b2CircleShape(32 / WORLD_SCALE);
            circleFix.shape = circlePly;
            circleFix.density = 1;
            circleFix.friction = 0.3;
            circleFix.restitution = 0.5;

            this.attach(bodyDef, circleFix);
        },

        attach: function(boxBodyDef, boxFixDef) {
            var b2body = this.world.CreateBody( boxBodyDef ); //ボディをworldに生成し…
            b2body.CreateFixture( boxFixDef ); //フィクスチャーを追加する
        },
    });

    // tm.define("tm.box2d.CircleShape", {
    //     superClass: "tm.display.CanvasElement",

    // });


})();

