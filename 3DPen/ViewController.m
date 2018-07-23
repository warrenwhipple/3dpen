//
//  ViewController.m
//  3DPen
//
//  Created by Warren Whipple on 3/17/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#import "ViewController.h"

#define MAX_VERTICES 1000

@interface ViewController()
{
    //GLKVector3 rotationD, rotationV, penD, penV, penA;
    float timeSinceLastPenDraw;
    BOOL penDrawing, tap;
    char colorRotation;
    int vertexCount;
    CMMotionManager *motionManager;
    NSOperationQueue *opQ;
    NSOperation *motionHandler;
    NSTimeInterval lastPenPositionTimestamp;
    float t3, t2, t1, t0;
    GLKVector3 a2, a1, a0, v3, v2, v1, d2;
    GLKVector3 *vertices;
    GLKVector4 *vertexColors;
    float _maxX, _maxY, _maxZ, _minX, _minY, _minZ;
}
@property (strong, nonatomic) EAGLContext *context;
@property (strong, nonatomic) GLKBaseEffect *effect;
@property (strong, nonatomic) NSMutableData *vertexData, *vertexColorData;
@end

@implementation ViewController
@synthesize context, effect, vertexData, vertexColorData;

#pragma mark - View lifecycle

-(void)viewDidLoad {
    [super viewDidLoad];
    [self setupPen];
    [self setupGL];
}

//-(void)viewDidUnload {
//    [super viewDidUnload];
//    [self tearDownGL];
//    [self tearDownPen];
//}

-(void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];    
    // Release any cached data, images, etc that aren't in use.
}

//-(BOOL)shouldAutorotateToInterfaceOrientation:(UIInterfaceOrientation)interfaceOrientation {
//    return (interfaceOrientation == UIInterfaceOrientationPortrait);
//}

#pragma mark - Pen Lifecycle

-(void)setupPen
{
    motionManager = [[CMMotionManager alloc] init];
    motionManager.deviceMotionUpdateInterval = 0.001;
    //motionManager.accelerometerUpdateInterval = 0.001;
    //motionManager.gyroUpdateInterval = 0.001;
    //NSLog(@"%f, %f", motionManager.accelerometerUpdateInterval, motionManager.gyroUpdateInterval);
    if (motionManager.isDeviceMotionAvailable) {
        [motionManager startDeviceMotionUpdatesUsingReferenceFrame:CMAttitudeReferenceFrameXArbitraryZVertical
                                                           toQueue:[NSOperationQueue currentQueue]
                                                       withHandler:^(CMDeviceMotion *deviceMotion, NSError *error) {
                                                           [self updatePenPositionWithDeviceMotion:deviceMotion];
                                                       }];
        /*
         [motionManager startAccelerometerUpdatesToQueue:[NSOperationQueue currentQueue]
                                            withHandler:^(CMAccelerometerData *accelerometerData, NSError *error) {[self updateWithAccelerometerData:accelerometerData];}];
         */
    } else NSLog(@"DeviceMotion is not available");
    
    self.vertexData = [NSMutableData dataWithLength:sizeof(GLKVector3)*MAX_VERTICES];
    vertices = [self.vertexData mutableBytes];
    self.vertexColorData = [NSMutableData dataWithLength:sizeof(GLKVector4)*MAX_VERTICES*2];
    vertexColors = [self.vertexColorData mutableBytes];
    vertexCount = 0;
    timeSinceLastPenDraw = 0.0;
    [self drawLogo];
}

-(void)tearDownPen {
    if ([motionManager isDeviceMotionActive]) [motionManager stopDeviceMotionUpdates];
}

-(void)updateWithAccelerometerData:(CMAccelerometerData *)a
{
    BOOL shouldPrint = NO;
    if (a.acceleration.x > _maxX) {_maxX = a.acceleration.x; shouldPrint = YES;}
    if (a.acceleration.y > _maxY) {_maxY = a.acceleration.y; shouldPrint = YES;}
    if (a.acceleration.z > _maxZ) {_maxZ = a.acceleration.z; shouldPrint = YES;}
    if (a.acceleration.x < _minX) {_minX = a.acceleration.x; shouldPrint = YES;}
    if (a.acceleration.y < _minY) {_minY = a.acceleration.y; shouldPrint = YES;}
    if (a.acceleration.z < _minZ) {_minZ = a.acceleration.z; shouldPrint = YES;}
    if (shouldPrint) printf("x: %f %f\ny: %f %f\nz: %f %f\n\n", _minX, _maxX, _minY, _maxY, _minZ, _maxZ);
}

-(void)updatePenPositionWithDeviceMotion:(CMDeviceMotion *)deviceMotion {
    CMAcceleration ua = deviceMotion.userAcceleration;
    CMRotationMatrix rm = deviceMotion.attitude.rotationMatrix;
    t3=t2; t2=t1; t1=t0;
    t0 = deviceMotion.timestamp;
    printf("%f %f\n", t0-t1, ua.x);
    a2=a1; a1=a0;
    a0 = GLKMatrix3MultiplyVector3(GLKMatrix3Make(rm.m11, rm.m12, rm.m13,
                                                  rm.m21, rm.m22, rm.m23,
                                                  rm.m31, rm.m32, rm.m33),
                                   GLKVector3Make(-ua.x, -ua.y, -ua.z));
    v3=v2; v2=v1;
    v1.x += (t0-t2)*(a2.x+(4.0*a1.x)+a0.x)/6.0;
    v1.y += (t0-t2)*(a2.y+(4.0*a1.y)+a0.y)/6.0;
    v1.z += (t0-t2)*(a2.z+(4.0*a1.z)+a0.z)/6.0;
    //v1.x *= 0.98;
    //v1.y *= 0.98;
    //v1.z *= 0.98;    
    d2.x += (t1-t3)*(v3.x+(4.0*v2.x)+v1.x)/6.0;
    d2.y += (t1-t3)*(v3.y+(4.0*v2.y)+v1.y)/6.0;
    d2.z += (t1-t3)*(v3.z+(4.0*v2.z)+v1.z)/6.0;
}

/*-(void)updatePenPositionWithDeviceMotion:(CMDeviceMotion *)deviceMotion {
    CMAcceleration ua = deviceMotion.userAcceleration;
    //CMAcceleration g = deviceMotion.gravity;
    CMRotationMatrix rm = deviceMotion.attitude.rotationMatrix;
    NSTimeInterval dt = deviceMotion.timestamp - lastPenPositionTimestamp;
    GLKVector3 resA = GLKMatrix3MultiplyVector3(GLKMatrix3Make(rm.m11, rm.m12, rm.m13,
                                                               rm.m21, rm.m22, rm.m23,
                                                               rm.m31, rm.m32, rm.m33),
                                                GLKVector3Make(-ua.x, -ua.y, -ua.z));
    
    penA.x = (resA.x);
    penA.y = (resA.y);
    penA.z = (resA.z);
    penV.x += penA.x * dt;
    penV.y += penA.y * dt;
    penV.z += penA.z * dt;
    penD.x += penV.x * dt;
    penD.y += penV.y * dt;
    penD.z += penV.z * dt;
    lastPenPositionTimestamp = deviceMotion.timestamp;
}*/ // old updatePenPosition

-(void)updatePenDrawing {
    if (penDrawing) [self addNewVertexX:d2.x*20 y:d2.y*20 z:d2.z*20];
    /*
    if (penDrawing) {
        vertices[0] = GLKVector3Make(0, 0, 0);
        vertices[1] = GLKVector3Make(penA.x*5, penA.y*5, penA.z*5);
        vertexColors[0] = GLKVector4Make(0, 0, 1, 1);
        vertexColors[1] = GLKVector4Make(1, 0, 0, 1);
        vertexCount = 2;
    }
    */
}

-(void)addNewVertexX:(float)x y:(float)y z:(float)z r:(float)r g:(float)g b:(float)b {
    if (vertexCount<MAX_VERTICES) {
        vertices[vertexCount] = GLKVector3Make(x,y,z);
        vertexColors[vertexCount] = GLKVector4Make(r,g,b,1);
        vertexCount++;
    }
}

-(void)addNewVertexX:(float)x y:(float)y z:(float)z {
    switch (colorRotation) {
        case'r': {[self addNewVertexX:x y:y z:z r:1 g:0 b:0]; colorRotation='y'; break;}
        case'y': {[self addNewVertexX:x y:y z:z r:1 g:1 b:0]; colorRotation='g'; break;}
        case'g': {[self addNewVertexX:x y:y z:z r:0 g:1 b:0]; colorRotation='c'; break;}
        case'c': {[self addNewVertexX:x y:y z:z r:0 g:1 b:1]; colorRotation='b'; break;}
        case'b': {[self addNewVertexX:x y:y z:z r:0 g:0 b:1]; colorRotation='m'; break;}
        case'm': {[self addNewVertexX:x y:y z:z r:1 g:0 b:1]; colorRotation='r'; break;}
        default: {[self addNewVertexX:x y:y z:z r:1 g:1 b:1]; break;}
    }
}

-(void)touchesBegan:(NSSet *)touches withEvent:(UIEvent *)event {
    //self.paused = !self.paused;
    penDrawing=YES;
    vertexCount=0;
    GLKVector3 zero = GLKVector3Make(0,0,0);
    v1=zero;
    v2=zero;
    v3=zero;
    d2=zero;
    /*penD.x=0;
    penD.y=0;
    penD.z=0;
    penV.x=0;
    penV.y=0;
    penV.z=0;
    rotationD.x = 0.0;
    rotationD.y = 0.0;
    rotationD.z = 0.0;
    rotationV.x = 0.0;
    rotationV.y = 0.0;
    rotationV.z = 0.0;*/ // old penUpdate variable reset
}

-(void)touchesEnded:(NSSet *)touches withEvent:(UIEvent *)event {
    penDrawing=NO;
}

-(void)drawLogo {
    vertexCount = 0;
    float q = 2.0/3.0;
    [self addNewVertexX:  0 y:  1 z: -q r:1.0 g:0.0 b:0.4];
    [self addNewVertexX: -q y:  1 z: -q r:1.0 g:0.0 b:0.2];
    [self addNewVertexX: -q y:  1 z:  q r:1.0 g:0.0 b:0.0]; // r
    [self addNewVertexX:  q y:  1 z:  q r:1.0 g:0.2 b:0.0];
    [self addNewVertexX:  q y:  1 z:  0 r:1.0 g:0.4 b:0.0];
    [self addNewVertexX:  1 y:  q z:  0 r:1.0 g:0.6 b:0.0];
    [self addNewVertexX:  1 y:  q z: -q r:1.0 g:0.8 b:0.0];
    [self addNewVertexX:  1 y: -q z: -q r:1.0 g:1.0 b:0.0]; // y
    [self addNewVertexX:  1 y: -q z:  q r:0.8 g:1.0 b:0.0];
    [self addNewVertexX:  1 y:  0 z:  q r:0.6 g:1.0 b:0.0];
    [self addNewVertexX:  q y:  0 z:  1 r:0.4 g:1.0 b:0.0];
    [self addNewVertexX:  q y:  q z:  1 r:0.2 g:1.0 b:0.0];
    [self addNewVertexX: -q y:  q z:  1 r:0.0 g:1.0 b:0.0]; // g
    [self addNewVertexX: -q y: -q z:  1 r:0.0 g:1.0 b:0.2];
    [self addNewVertexX:  0 y: -q z:  1 r:0.0 g:1.0 b:0.4];
    [self addNewVertexX:  0 y: -1 z:  q r:0.0 g:1.0 b:0.6];
    [self addNewVertexX:  q y: -1 z:  q r:0.0 g:1.0 b:0.8];
    [self addNewVertexX:  q y: -1 z: -q r:0.0 g:1.0 b:1.0]; // c
    [self addNewVertexX: -q y: -1 z: -q r:0.0 g:0.8 b:1.0];
    [self addNewVertexX: -q y: -1 z:  0 r:0.0 g:0.6 b:1.0];
    [self addNewVertexX: -1 y: -q z:  0 r:0.0 g:0.4 b:1.0];
    [self addNewVertexX: -1 y: -q z:  q r:0.0 g:0.2 b:1.0];
    [self addNewVertexX: -1 y:  q z:  q r:0.0 g:0.0 b:1.0]; // b
    [self addNewVertexX: -1 y:  q z: -q r:0.2 g:0.0 b:1.0];
    [self addNewVertexX: -1 y:  0 z: -q r:0.4 g:0.0 b:1.0];
    [self addNewVertexX: -q y:  0 z: -1 r:0.6 g:0.0 b:1.0];
    [self addNewVertexX: -q y: -q z: -1 r:0.8 g:0.0 b:1.0];
    [self addNewVertexX:  q y: -q z: -1 r:1.0 g:0.0 b:1.0]; // m
    [self addNewVertexX:  q y:  q z: -1 r:1.0 g:0.0 b:0.8];
    [self addNewVertexX:  0 y:  q z: -1 r:1.0 g:0.0 b:0.6];
    [self addNewVertexX:  0 y:  1 z: -q r:1.0 g:0.0 b:0.4];
    colorRotation = 'r';    
}

#pragma mark GL Lifecycle

-(void)setupGL {
    self.context = [[EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES2];
    GLKView *view = (GLKView *)self.view;
    view.context = self.context;
    //view.drawableMultisample = GLKViewDrawableMultisample4X;
    //view.drawableDepthFormat = GLKViewDrawableDepthFormat24;
    [EAGLContext setCurrentContext:self.context];
    self.effect = [[GLKBaseEffect alloc] init];
    //glEnable(GL_DEPTH_TEST);
    /*rotationV.x = 0.2;
    rotationV.y = 1.0;
    rotationV.z = 0.0;*/
}

-(void)tearDownGL {
    [EAGLContext setCurrentContext:self.context];
    self.effect = nil;
    if ([EAGLContext currentContext] == self.context) [EAGLContext setCurrentContext:nil];
    self.context = nil;
}

-(void)updateGL {
    /*rotationD.x += rotationV.x * self.timeSinceLastUpdate;
    rotationD.y += rotationV.y * self.timeSinceLastUpdate;
    rotationD.z += rotationV.z * self.timeSinceLastUpdate;
    if      (rotationD.x > 2*M_PI) rotationD.x -= 2*M_PI;
    else if (rotationD.x < 0)      rotationD.x += 2*M_PI;
    if      (rotationD.y > 2*M_PI) rotationD.y -= 2*M_PI;
    else if (rotationD.y < 0)      rotationD.y += 2*M_PI;
    if      (rotationD.z > 2*M_PI) rotationD.z -= 2*M_PI;
    else if (rotationD.z < 0)      rotationD.z += 2*M_PI;*/
    
    float aspect = fabs(self.view.bounds.size.width / self.view.bounds.size.height);
    GLKMatrix4 projectionMatrix = GLKMatrix4MakePerspective(GLKMathDegreesToRadians(65.0f), aspect, 0.0f, 100.0f);
    self.effect.transform.projectionMatrix = projectionMatrix;
    
    //GLKMatrix4 modelViewMatrix = GLKMatrix4MakeTranslation(0.0f, 0.0f, -10.0f);
    /*modelViewMatrix = GLKMatrix4Rotate(modelViewMatrix, rotationD.x, 1, 0, 0);
    modelViewMatrix = GLKMatrix4Rotate(modelViewMatrix, rotationD.y, 0, 1, 0);
    modelViewMatrix = GLKMatrix4Rotate(modelViewMatrix, rotationD.z, 0, 0, 1);*/
    CMRotationMatrix rm = motionManager.deviceMotion.attitude.rotationMatrix;
    GLKMatrix4 modelViewMatrix = GLKMatrix4Make(rm.m11, rm.m21, rm.m31, 0,
                                                rm.m12, rm.m22, rm.m32, 0,
                                                rm.m13, rm.m23, rm.m33, 0,
                                                     0,      0,  -10.0, 1);
    self.effect.transform.modelviewMatrix = modelViewMatrix;
}

#pragma mark - GLKViewDelegate methods

-(void)glkView:(GLKView *)view drawInRect:(CGRect)rect {
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    [self.effect prepareToDraw];
    glEnableVertexAttribArray(GLKVertexAttribPosition);
    glVertexAttribPointer(GLKVertexAttribPosition, 3, GL_FLOAT, GL_FALSE, 0, vertices);
    glEnableVertexAttribArray(GLKVertexAttribColor);
    glVertexAttribPointer(GLKVertexAttribColor, 4, GL_FLOAT, GL_FALSE, 0, vertexColors);
    glLineWidth(11.0);
    glDrawArrays(GL_LINE_STRIP, 0, vertexCount);
}

#pragma mark - GLKViewControllerDelegate methods

-(void)update {
    [self updatePenDrawing];
    [self updateGL];
}

@end
