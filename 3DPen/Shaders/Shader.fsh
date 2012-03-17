//
//  Shader.fsh
//  3DPen
//
//  Created by Warren Whipple on 3/17/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

varying lowp vec4 colorVarying;

void main()
{
    gl_FragColor = colorVarying;
}
