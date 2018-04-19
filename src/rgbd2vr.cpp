#include <ros/console.h>

#include "rgbd2vr.hpp"

using namespace cv;
using namespace std;


/**
  * Taken from https://stackoverflow.com/questions/28562401/resize-an-image-to-a-square-but-keep-aspect-ratio-c-opencv
  */
cv::Mat RGBD2VR::resizeKeepAspectRatio(const cv::Mat &input, const cv::Size &dstSize, const cv::Scalar &bgcolor)
{
    if(input.empty()) return input;

    cv::Mat output;

    double h1 = dstSize.width * (input.rows/(double)input.cols);
    double w2 = dstSize.height * (input.cols/(double)input.rows);

    // initially no borders
    int top = 0;
    int down = 0;
    int left = 0;
    int right = 0;
    if( h1 <= dstSize.height)
    {
        // only vertical borders
        top = (dstSize.height - h1) / 2;
        down = top;
        cv::resize( input, output, cv::Size(dstSize.width, h1));
    }
    else
    {
        // only horizontal borders
        left = (dstSize.width - w2) / 2;
        right = left;
        cv::resize( input, output, cv::Size(w2, dstSize.height));
    }

    return output;
}

RGBD2VR::RGBD2VR( int argc, char *argv[] )
    : m_pCompanionWindow(NULL)
    , m_pContext(NULL)
    , m_nCompanionWindowWidth( 640 )
    , m_nCompanionWindowHeight( 320 )
    , baseline_scale(1.f)
    , m_BackgroundProgramID( 0 )
    , m_unSceneProgramID( 0 )
    , m_unCompanionWindowProgramID( 0 )
    , m_unControllerTransformProgramID( 0 )
    , m_unRenderModelProgramID( 0 )
    , m_pHMD( NULL )
    , shutdownRequested( false )
    , m_pRenderModels( NULL )
    , m_bDebugOpenGL( false )
    , m_bVerbose( false )
    , m_bPerf( false )
    , m_bVblank( false )
    , m_bGlFinishHack( true )
    , m_glControllerVertBuffer( 0 )
    , m_unControllerVAO( 0 )
    , m_BackgroundVAO( 0 )
    , m_unSceneVAO( 0 )
    , m_nSceneMatrixLocation( -1 )
    , m_nControllerMatrixLocation( -1 )
    , m_nRenderModelMatrixLocation( -1 )
    , m_iTrackedControllerCount( 0 )
    , m_iTrackedControllerCount_Last( -1 )
    , m_iValidPoseCount( 0 )
    , m_iValidPoseCount_Last( -1 )
      , m_iSceneVolumeInit( 20 )
    , m_strPoseClasses("")
      , m_bShowCubes( true )
{

    uint8_t camId = 0;

    for( int i = 1; i < argc; i++ )
    {
        if( !strcmp( argv[i], "--gldebug" ) )
        {
            m_bDebugOpenGL = true;
        }
        else if( !strcmp( argv[i], "--verbose" ) )
        {
            m_bVerbose = true;
        }
        else if( !strcmp( argv[i], "--novblank" ) )
        {
            m_bVblank = false;
        }
        else if( !strcmp( argv[i], "--noglfinishhack" ) )
        {
            m_bGlFinishHack = false;
        }
        else if ( !strcmp( argv[i], "--cubevolume" ) && ( argc > i + 1 ) && ( *argv[ i + 1 ] != '-' ) )
        {
            m_iSceneVolumeInit = atoi( argv[ i + 1 ] );
            i++;
        }
        else if ( !strcmp( argv[i], "--camera" ) && ( argc > i + 1 ) )
        {
            camId = atoi( argv[ i + 1 ] );
            i++;
        }
    }

    // other initialization tasks are done in BInit
    memset(m_rDevClassChar, 0, sizeof(m_rDevClassChar));
};


//-----------------------------------------------------------------------------
// Purpose: Destructor
//-----------------------------------------------------------------------------
RGBD2VR::~RGBD2VR()
{
    // work is done in shutdown
    ROS_INFO( "shutdown" );
}


//-----------------------------------------------------------------------------
// Purpose: Helper to get a string from a tracked device property and turn it
//			into a std::string
//-----------------------------------------------------------------------------
std::string GetTrackedDeviceString( vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = NULL )
{
    uint32_t unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty( unDevice, prop, NULL, 0, peError );
    if( unRequiredBufferLen == 0 )
        return "";

    char *pchBuffer = new char[ unRequiredBufferLen ];
    unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty( unDevice, prop, pchBuffer, unRequiredBufferLen, peError );
    std::string sResult = pchBuffer;
    delete [] pchBuffer;
    return sResult;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool RGBD2VR::BInit()
{
    if ( SDL_Init( SDL_INIT_VIDEO | SDL_INIT_TIMER ) < 0 )
    {
        printf("%s - SDL could not initialize! SDL Error: %s\n", __FUNCTION__, SDL_GetError());
        return false;
    }

    // Loading the SteamVR Runtime
    vr::EVRInitError eError = vr::VRInitError_None;
    m_pHMD = vr::VR_Init( &eError, vr::VRApplication_Scene );

    if ( eError != vr::VRInitError_None )
    {
        m_pHMD = NULL;
        char buf[1024];
        snprintf( buf, sizeof( buf ), "Unable to init VR runtime: %s", vr::VR_GetVRInitErrorAsEnglishDescription( eError ) );
        SDL_ShowSimpleMessageBox( SDL_MESSAGEBOX_ERROR, "VR_Init Failed", buf, NULL );
        return false;
    }


    m_pRenderModels = (vr::IVRRenderModels *)vr::VR_GetGenericInterface( vr::IVRRenderModels_Version, &eError );
    if( !m_pRenderModels )
    {
        m_pHMD = NULL;
        vr::VR_Shutdown();

        char buf[1024];
        snprintf( buf, sizeof( buf ), "Unable to get render model interface: %s", vr::VR_GetVRInitErrorAsEnglishDescription( eError ) );
        SDL_ShowSimpleMessageBox( SDL_MESSAGEBOX_ERROR, "VR_Init Failed", buf, NULL );
        return false;
    }

    int nWindowPosX = 700;
    int nWindowPosY = 100;
    Uint32 unWindowFlags = SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN;

    SDL_GL_SetAttribute( SDL_GL_CONTEXT_MAJOR_VERSION, 4 );
    SDL_GL_SetAttribute( SDL_GL_CONTEXT_MINOR_VERSION, 1 );
    //SDL_GL_SetAttribute( SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_COMPATIBILITY );
    SDL_GL_SetAttribute( SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE );

    SDL_GL_SetAttribute( SDL_GL_MULTISAMPLEBUFFERS, 0 );
    SDL_GL_SetAttribute( SDL_GL_MULTISAMPLESAMPLES, 0 );
    if( m_bDebugOpenGL )
        SDL_GL_SetAttribute( SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG );

    m_pCompanionWindow = SDL_CreateWindow( "hellovr", nWindowPosX, nWindowPosY, m_nCompanionWindowWidth, m_nCompanionWindowHeight, unWindowFlags );
    if (m_pCompanionWindow == NULL)
    {
        printf( "%s - Window could not be created! SDL Error: %s\n", __FUNCTION__, SDL_GetError() );
        return false;
    }

    m_pContext = SDL_GL_CreateContext(m_pCompanionWindow);
    if (m_pContext == NULL)
    {
        printf( "%s - OpenGL context could not be created! SDL Error: %s\n", __FUNCTION__, SDL_GetError() );
        return false;
    }

    glewExperimental = GL_TRUE;
    GLenum nGlewError = glewInit();
    if (nGlewError != GLEW_OK)
    {
        printf( "%s - Error initializing GLEW! %s\n", __FUNCTION__, glewGetErrorString( nGlewError ) );
        return false;
    }
    glGetError(); // to clear the error caused deep in GLEW

    if ( SDL_GL_SetSwapInterval( m_bVblank ? 1 : 0 ) < 0 )
    {
        printf( "%s - Warning: Unable to set VSync! SDL Error: %s\n", __FUNCTION__, SDL_GetError() );
        return false;
    }


    m_strDriver = "No Driver";
    m_strDisplay = "No Display";

    m_strDriver = GetTrackedDeviceString( m_pHMD, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String );
    m_strDisplay = GetTrackedDeviceString( m_pHMD, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SerialNumber_String );

    std::string strWindowTitle = "hellovr - " + m_strDriver + " " + m_strDisplay;
    SDL_SetWindowTitle( m_pCompanionWindow, strWindowTitle.c_str() );

    // cube array
    m_iSceneVolumeWidth = m_iSceneVolumeInit;
    m_iSceneVolumeHeight = m_iSceneVolumeInit;
    m_iSceneVolumeDepth = m_iSceneVolumeInit;

    m_fScale = 0.3f;
    m_fScaleSpacing = 4.0f;

    m_fNearClip = 0.1f;
    m_fFarClip = 30.0f;

    //m_iTexture = 0;
    leftEyeVideoTexture = 0;
    rightEyeVideoTexture = 0;

    m_uiVertcount = 0;

    // 		m_MillisecondsTimer.start(1, this);
    // 		m_SecondsTimer.start(1000, this);

    if (!BInitGL())
    {
        printf("%s - Unable to initialize OpenGL!\n", __FUNCTION__);
        return false;
    }

    if (!BInitCompositor())
    {
        printf("%s - Failed to initialize VR Compositor!\n", __FUNCTION__);
        return false;
    }

    return true;
}


//-----------------------------------------------------------------------------
// Purpose: Outputs the string in message to debugging output.
//          All other parameters are ignored.
//          Does not return any meaningful value or reference.
//-----------------------------------------------------------------------------
void DebugCallback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const char* message, const void* userParam)
{
    ROS_INFO( "GL Error: %s\n", message );
}


//-----------------------------------------------------------------------------
// Purpose: Initialize OpenGL. Returns true if OpenGL has been successfully
//          initialized, false if shaders could not be created.
//          If failure occurred in a module other than shaders, the function
//          may return true or throw an error. 
//-----------------------------------------------------------------------------
bool RGBD2VR::BInitGL()
{
    if( m_bDebugOpenGL )
    {
        glDebugMessageCallback( (GLDEBUGPROC)DebugCallback, nullptr);
        glDebugMessageControl( GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE );
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
    }

    if( !CreateAllShaders() )
        return false;

    initializeTextures();
    SetupScene();
    SetupCameras();
	SetupStereoRenderTargets();
    SetupRenderModels();

    return true;
}


//-----------------------------------------------------------------------------
// Purpose: Initialize Compositor. Returns true if the compositor was
//          successfully initialized, false otherwise.
//-----------------------------------------------------------------------------
bool RGBD2VR::BInitCompositor()
{
    vr::EVRInitError peError = vr::VRInitError_None;

    if ( !vr::VRCompositor() )
    {
        printf( "Compositor initialization failed. See log file for details\n" );
        return false;
    }

    return true;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void RGBD2VR::shutdown()
{
    if( m_pHMD )
    {
        vr::VR_Shutdown();
        m_pHMD = NULL;
    }

    for( std::vector< CGLRenderModel * >::iterator i = m_vecRenderModels.begin(); i != m_vecRenderModels.end(); i++ )
    {
        delete (*i);
    }
    m_vecRenderModels.clear();

    if( m_pContext )
    {
        if( m_bDebugOpenGL )
        {
            glDebugMessageControl( GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_FALSE );
            glDebugMessageCallback(nullptr, nullptr);
        }
        glDeleteBuffers(1, &m_glBackgroundVertBuffer);
        if ( m_BackgroundProgramID )
        {
            glDeleteProgram( m_BackgroundProgramID );
        }
        glDeleteBuffers(1, &m_glSceneVertBuffer);
        if ( m_unSceneProgramID )
        {
            glDeleteProgram( m_unSceneProgramID );
        }
        if ( m_unControllerTransformProgramID )
        {
            glDeleteProgram( m_unControllerTransformProgramID );
        }
        if ( m_unRenderModelProgramID )
        {
            glDeleteProgram( m_unRenderModelProgramID );
        }
        if ( m_unCompanionWindowProgramID )
        {
            glDeleteProgram( m_unCompanionWindowProgramID );
        }

        glDeleteRenderbuffers( 1, &leftEyeDesc.m_nDepthBufferId );
        glDeleteTextures( 1, &leftEyeDesc.m_nRenderTextureId );
        glDeleteFramebuffers( 1, &leftEyeDesc.m_nRenderFramebufferId );
        glDeleteTextures( 1, &leftEyeDesc.m_nResolveTextureId );
        glDeleteFramebuffers( 1, &leftEyeDesc.m_nResolveFramebufferId );

        glDeleteRenderbuffers( 1, &rightEyeDesc.m_nDepthBufferId );
        glDeleteTextures( 1, &rightEyeDesc.m_nRenderTextureId );
        glDeleteFramebuffers( 1, &rightEyeDesc.m_nRenderFramebufferId );
        glDeleteTextures( 1, &rightEyeDesc.m_nResolveTextureId );
        glDeleteFramebuffers( 1, &rightEyeDesc.m_nResolveFramebufferId );

        if( m_unCompanionWindowVAO != 0 )
        {
            glDeleteVertexArrays( 1, &m_unCompanionWindowVAO );
        }
        if( m_BackgroundVAO != 0 )
        {
            glDeleteVertexArrays( 1, &m_BackgroundVAO );
        }
        if( m_unSceneVAO != 0 )
        {
            glDeleteVertexArrays( 1, &m_unSceneVAO );
        }
        if( m_unControllerVAO != 0 )
        {
            glDeleteVertexArrays( 1, &m_unControllerVAO );
        }
    }

    if( m_pCompanionWindow )
    {
        SDL_DestroyWindow(m_pCompanionWindow);
        m_pCompanionWindow = NULL;
    }

    SDL_Quit();
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool RGBD2VR::HandleInput()
{
    SDL_Event sdlEvent;
    bool bRet = false;

    while ( SDL_PollEvent( &sdlEvent ) != 0 )
    {
        if ( sdlEvent.type == SDL_QUIT )
        {
            bRet = true;
        }
        else if ( sdlEvent.type == SDL_KEYDOWN )
        {
            if ( sdlEvent.key.keysym.sym == SDLK_ESCAPE 
                    || sdlEvent.key.keysym.sym == SDLK_q )
            {
                bRet = true;
            }
            if( sdlEvent.key.keysym.sym == SDLK_c )
            {
                m_bShowCubes = !m_bShowCubes;
            }
            if( sdlEvent.key.keysym.sym == SDLK_LEFT )
            {
                baseline_scale *= 0.9;
            }
            if( sdlEvent.key.keysym.sym == SDLK_RIGHT )
            {
                baseline_scale *= 1.1;
            }
        }
    }

    // Process SteamVR events
    vr::VREvent_t event;
    while( m_pHMD->PollNextEvent( &event, sizeof( event ) ) )
    {
        ProcessVREvent( event );
    }

    // Process SteamVR controller state
    for( vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++ )
    {
        vr::VRControllerState_t state;
        if( m_pHMD->GetControllerState( unDevice, &state, sizeof(state) ) )
        {
            m_rbShowTrackedDevice[ unDevice ] = state.ulButtonPressed == 0;
        }
    }

    return bRet;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void RGBD2VR::start()
{
    SDL_StartTextInput();
    SDL_ShowCursor( SDL_DISABLE );
}

void RGBD2VR::stop()
{
    SDL_StopTextInput();
}

void RGBD2VR::step() {
    shutdownRequested = HandleInput();
    RenderFrame();
}

//-----------------------------------------------------------------------------
// Purpose: Processes a single VR event
//-----------------------------------------------------------------------------
void RGBD2VR::ProcessVREvent( const vr::VREvent_t & event )
{
    switch( event.eventType )
    {
        case vr::VREvent_TrackedDeviceActivated:
            {
                SetupRenderModelForTrackedDevice( event.trackedDeviceIndex );
                ROS_INFO( "Device %u attached. Setting up render model.\n", event.trackedDeviceIndex );
            }
            break;
        case vr::VREvent_TrackedDeviceDeactivated:
            {
                ROS_INFO( "Device %u detached.\n", event.trackedDeviceIndex );
            }
            break;
        case vr::VREvent_TrackedDeviceUpdated:
            {
                ROS_INFO( "Device %u updated.\n", event.trackedDeviceIndex );
            }
            break;
    }
}

tuple<Mat, Mat> RGBD2VR::fakeStereo(Mat rgb, Mat disparity) {

    //Mat leftEye = Mat::zeros(rgb.rows, rgb.cols, rgb.type());
    //Mat rightEye = Mat::zeros(rgb.rows, rgb.cols, rgb.type());
    Mat leftEye = rgb.clone();
    Mat rightEye = rgb.clone();

    // fill the holes in the disparity map -> inpainting too slow!! openvr refuses to launch the app
    //auto mask = (disparity == 255);
    //Mat holeFilling;
    //cv::inpaint(disparity, mask, holeFilling, 1, cv::INPAINT_NS);
    //holeFilling.copyTo(disparity, mask);
    //blur(disparity, disparity, {5,5});

    /*
    for (int x = 0; x < rgb.cols; x++) {
        for (int y = 0; y < rgb.rows; y++) {
            auto d = disparity.at<uint8_t>(Point(x,y)) * baseline_scale;
            if (d < 255) { // 255 is a sentinel value meaning no matching found
                auto color = rgb.at<Vec3b>(Point(x,y));
                if (x + d < rgb.cols) {
                    leftEye.at<Vec3b>(Point(x + d, y)) = color;
                }
                if (x > d) {
                    rightEye.at<Vec3b>(Point(x - d, y)) = color;
                }
            }
        }
    }
    */

    //imshow( "left eye", leftEye );
    //imshow( "right eye", rightEye );

    //Mat anaglyph = Mat::zeros(rgb.rows, rgb.cols, rgb.type());
    //Mat in[] = { leftEye, rightEye };
    //int from_to[] = { 0,0, 5,2 };
    //mixChannels( in, 2, &anaglyph, 1, from_to, 2 );
    //imshow( "anaglyph", anaglyph );
    
    return {leftEye, rightEye};
}

/**
 * rgb: CV_8UC3, RGB
 * depth: CV_16U
 */
void RGBD2VR::setNextBackgroundFrames(Mat raw_rgb, Mat raw_depth)
{

    Mat flippedRGB, flippedDepth;
    cv::flip(raw_rgb, flippedRGB, 0);
    cv::flip(raw_depth, flippedDepth, 0);

    auto rgb = resizeKeepAspectRatio(flippedRGB, Size(m_nRenderWidth, m_nRenderHeight));
    auto depth = resizeKeepAspectRatio(flippedDepth, Size(m_nRenderWidth, m_nRenderHeight));

    // < 800
    Mat disparity = depth.clone();
    depth.convertTo( disparity, CV_8UC1, 1./32 ); // in default conf, 32 unit = 1 px of disparity. cf https://github.com/IntelRealSense/librealsense/blob/v1.12.1/doc/projection.md#depth-image-formats

    Mat leftEye, rightEye;
    tie(leftEye, rightEye) = fakeStereo(rgb, disparity);

    // left

    glBindTexture( GL_TEXTURE_2D, leftEyeVideoTexture );
    glTexImage2D(GL_TEXTURE_2D,     // Type of texture
                    0,                 // Pyramid level (for mip-mapping) - 0 is the top level
                    GL_RGB,            // Internal colour format to convert to
                    m_nRenderWidth,          // Image width  i.e. 640 for Kinect in standard mode
                    m_nRenderHeight,          // Image height i.e. 480 for Kinect in standard mode
                    0,                 // Border width in pixels (can either be 1 or 0)
                    GL_BGR, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                    GL_UNSIGNED_BYTE,  // Image data type
                    leftEye.ptr());        // The actual image data itself

    glGenerateMipmap(GL_TEXTURE_2D);

    glBindTexture( GL_TEXTURE_2D, 0 );


    // right
    //glBindTexture( GL_TEXTURE_2D, rightEyeVideoTexture );
    //glTexImage2D(GL_TEXTURE_2D,     // Type of texture
    //                0,                 // Pyramid level (for mip-mapping) - 0 is the top level
    //                GL_RGB,            // Internal colour format to convert to
    //                m_nRenderWidth,          // Image width  i.e. 640 for Kinect in standard mode
    //                m_nRenderHeight,          // Image height i.e. 480 for Kinect in standard mode
    //                0,                 // Border width in pixels (can either be 1 or 0)
    //                GL_BGR, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
    //                GL_UNSIGNED_BYTE,  // Image data type
    //                rightEye.ptr());        // The actual image data itself
    //glGenerateMipmap(GL_TEXTURE_2D);
    //glBindTexture( GL_TEXTURE_2D, 0 );
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void RGBD2VR::RenderFrame()
{
    // for now as fast as possible
    //if ( m_pHMD )
    //{


        RenderControllerAxes();
        RenderStereoTargets();

        vr::Texture_t leftEyeTexture = {(void*)(uintptr_t)leftEyeDesc.m_nResolveTextureId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };

        vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture );

        vr::Texture_t rightEyeTexture = {(void*)(uintptr_t)rightEyeDesc.m_nResolveTextureId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
        vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture );
    //}

    if ( m_bVblank && m_bGlFinishHack )
    {
        //$ HACKHACK. From gpuview profiling, it looks like there is a bug where two renders and a present
        // happen right before and after the vsync causing all kinds of jittering issues. This glFinish()
        // appears to clear that up. Temporary fix while I try to get nvidia to investigate this problem.
        // 1/29/2014 mikesart
        glFinish();
    }

    // SwapWindow
    {
        SDL_GL_SwapWindow( m_pCompanionWindow );
    }

    // Clear
    {
        // We want to make sure the glFinish waits for the entire present to complete, not just the submission
        // of the command. So, we do a clear here right here so the glFinish will wait fully for the swap.
        glClearColor( 0, 0, 0, 1 );
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    }

    // Flush and wait for swap.
    if ( m_bVblank )
    {
        glFlush();
        glFinish();
    }

    // Spew out the controller and pose count whenever they change.
    if ( m_iTrackedControllerCount != m_iTrackedControllerCount_Last || m_iValidPoseCount != m_iValidPoseCount_Last )
    {
        m_iValidPoseCount_Last = m_iValidPoseCount;
        m_iTrackedControllerCount_Last = m_iTrackedControllerCount;

        ROS_INFO( "PoseCount:%d(%s) Controllers:%d\n", m_iValidPoseCount, m_strPoseClasses.c_str(), m_iTrackedControllerCount );
    }

    UpdateHMDMatrixPose();
}


//-----------------------------------------------------------------------------
// Purpose: Compiles a GL shader program and returns the handle. Returns 0 if
//			the shader couldn't be compiled for some reason.
//-----------------------------------------------------------------------------
GLuint RGBD2VR::CompileGLShader( const char *pchShaderName, const char *pchVertexShader, const char *pchFragmentShader )
{
    GLuint unProgramID = glCreateProgram();

    GLuint nSceneVertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource( nSceneVertexShader, 1, &pchVertexShader, NULL);
    glCompileShader( nSceneVertexShader );

    GLint vShaderCompiled = GL_FALSE;
    glGetShaderiv( nSceneVertexShader, GL_COMPILE_STATUS, &vShaderCompiled);
    if ( vShaderCompiled != GL_TRUE)
    {
        ROS_INFO("%s - Unable to compile vertex shader %d!\n", pchShaderName, nSceneVertexShader);
        glDeleteProgram( unProgramID );
        glDeleteShader( nSceneVertexShader );
        return 0;
    }
    glAttachShader( unProgramID, nSceneVertexShader);
    glDeleteShader( nSceneVertexShader ); // the program hangs onto this once it's attached

    GLuint  nSceneFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource( nSceneFragmentShader, 1, &pchFragmentShader, NULL);
    glCompileShader( nSceneFragmentShader );

    GLint fShaderCompiled = GL_FALSE;
    glGetShaderiv( nSceneFragmentShader, GL_COMPILE_STATUS, &fShaderCompiled);
    if (fShaderCompiled != GL_TRUE)
    {
        ROS_INFO("%s - Unable to compile fragment shader %d!\n", pchShaderName, nSceneFragmentShader );
        glDeleteProgram( unProgramID );
        glDeleteShader( nSceneFragmentShader );
        return 0;	
    }

    glAttachShader( unProgramID, nSceneFragmentShader );
    glDeleteShader( nSceneFragmentShader ); // the program hangs onto this once it's attached

    glLinkProgram( unProgramID );

    GLint programSuccess = GL_TRUE;
    glGetProgramiv( unProgramID, GL_LINK_STATUS, &programSuccess);
    if ( programSuccess != GL_TRUE )
    {
        ROS_INFO("%s - Error linking program %d!\n", pchShaderName, unProgramID);
        glDeleteProgram( unProgramID );
        return 0;
    }

    glUseProgram( unProgramID );
    glUseProgram( 0 );

    return unProgramID;
}


//-----------------------------------------------------------------------------
// Purpose: Creates all the shaders used by HelloVR SDL
//-----------------------------------------------------------------------------
bool RGBD2VR::CreateAllShaders()
{
    m_BackgroundProgramID = CompileGLShader( 
            "Background",

            // Vertex Shader
            "#version 410\n"
            "in vec4 position;\n"
            "in vec2 inputTextureCoordinate;\n"
            "out vec2 v2UVcoords;\n"
            "void main()\n"
            "{\n"
            "    gl_Position = position;\n"
            "    v2UVcoords = inputTextureCoordinate;\n"
            "}\n",
            // Fragment Shader
            "#version 410 core\n"
            "uniform sampler2D mytexture;\n"
            "in vec2 v2UVcoords;\n"
            "out vec4 outputColor;\n"
            "void main()\n"
            "{\n"
            "   outputColor = texture(mytexture, v2UVcoords);\n"
            "}\n"
            );

    m_unSceneProgramID = CompileGLShader( 
            "Scene",

            // Vertex Shader
            "#version 410\n"
            "uniform mat4 matrix;\n"
            "layout(location = 0) in vec4 position;\n"
            "layout(location = 1) in vec2 v2UVcoordsIn;\n"
            "layout(location = 2) in vec3 v3NormalIn;\n"
            "out vec2 v2UVcoords;\n"
            "void main()\n"
            "{\n"
            "	v2UVcoords = v2UVcoordsIn;\n"
            "	gl_Position = matrix * position;\n"
            "}\n",

            // Fragment Shader
            "#version 410 core\n"
            "uniform sampler2D mytexture;\n"
            "in vec2 v2UVcoords;\n"
            "out vec4 outputColor;\n"
            "void main()\n"
            "{\n"
            "   outputColor = texture(mytexture, v2UVcoords);\n"
            "}\n"
            );
    m_nSceneMatrixLocation = glGetUniformLocation( m_unSceneProgramID, "matrix" );
    if( m_nSceneMatrixLocation == -1 )
    {
        ROS_INFO( "Unable to find matrix uniform in scene shader\n" );
        return false;
    }

    m_unControllerTransformProgramID = CompileGLShader(
            "Controller",

            // vertex shader
            "#version 410\n"
            "uniform mat4 matrix;\n"
            "layout(location = 0) in vec4 position;\n"
            "layout(location = 1) in vec3 v3ColorIn;\n"
            "out vec4 v4Color;\n"
            "void main()\n"
            "{\n"
            "	v4Color.xyz = v3ColorIn; v4Color.a = 1.0;\n"
            "	gl_Position = matrix * position;\n"
            "}\n",

            // fragment shader
            "#version 410\n"
            "in vec4 v4Color;\n"
            "out vec4 outputColor;\n"
            "void main()\n"
            "{\n"
            "   outputColor = v4Color;\n"
            "}\n"
            );
    m_nControllerMatrixLocation = glGetUniformLocation( m_unControllerTransformProgramID, "matrix" );
    if( m_nControllerMatrixLocation == -1 )
    {
        ROS_INFO( "Unable to find matrix uniform in controller shader\n" );
        return false;
    }

    m_unRenderModelProgramID = CompileGLShader( 
            "render model",

            // vertex shader
            "#version 410\n"
            "uniform mat4 matrix;\n"
            "layout(location = 0) in vec4 position;\n"
            "layout(location = 1) in vec3 v3NormalIn;\n"
            "layout(location = 2) in vec2 v2TexCoordsIn;\n"
            "out vec2 v2TexCoord;\n"
            "void main()\n"
            "{\n"
            "	v2TexCoord = v2TexCoordsIn;\n"
            "	gl_Position = matrix * vec4(position.xyz, 1);\n"
            "}\n",

            //fragment shader
            "#version 410 core\n"
            "uniform sampler2D diffuse;\n"
            "in vec2 v2TexCoord;\n"
            "out vec4 outputColor;\n"
            "void main()\n"
            "{\n"
            "   outputColor = texture( diffuse, v2TexCoord);\n"
            "}\n"

            );
    m_nRenderModelMatrixLocation = glGetUniformLocation( m_unRenderModelProgramID, "matrix" );
    if( m_nRenderModelMatrixLocation == -1 )
    {
        ROS_INFO( "Unable to find matrix uniform in render model shader\n" );
        return false;
    }

    m_unCompanionWindowProgramID = CompileGLShader(
            "CompanionWindow",

            // vertex shader
            "#version 410 core\n"
            "layout(location = 0) in vec4 position;\n"
            "layout(location = 1) in vec2 v2UVIn;\n"
            "noperspective out vec2 v2UV;\n"
            "void main()\n"
            "{\n"
            "	v2UV = v2UVIn;\n"
            "	gl_Position = position;\n"
            "}\n",

            // fragment shader
            "#version 410 core\n"
            "uniform sampler2D mytexture;\n"
            "noperspective in vec2 v2UV;\n"
            "out vec4 outputColor;\n"
            "void main()\n"
            "{\n"
            "		outputColor = texture(mytexture, v2UV);\n"
            "}\n"
            );

    return m_BackgroundProgramID != 0 
        && m_unSceneProgramID != 0 
        && m_unControllerTransformProgramID != 0
        && m_unRenderModelProgramID != 0
        && m_unCompanionWindowProgramID != 0;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool RGBD2VR::initializeTextures()
{

    Mat defaultImage(m_nRenderWidth, m_nRenderHeight, CV_8UC3, Scalar(128,128,0));


    // left eye
    glGenTextures(1, &leftEyeVideoTexture );
    glBindTexture( GL_TEXTURE_2D, leftEyeVideoTexture );

    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );

    glTexImage2D(GL_TEXTURE_2D,     // Type of texture
                    0,                 // Pyramid level (for mip-mapping) - 0 is the top level
                    GL_RGB,            // Internal colour format to convert to
                    m_nRenderWidth,          // Image width  i.e. 640 for Kinect in standard mode
                    m_nRenderHeight,          // Image height i.e. 480 for Kinect in standard mode
                    0,                 // Border width in pixels (can either be 1 or 0)
                    GL_BGR, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                    GL_UNSIGNED_BYTE,  // Image data type
                    defaultImage.ptr());        // The actual image data itself

 
    GLfloat fLargest;
    glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &fLargest);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, fLargest);

    glBindTexture( GL_TEXTURE_2D, 0 );


    // right eye
    glGenTextures(1, &rightEyeVideoTexture );
    glBindTexture( GL_TEXTURE_2D, rightEyeVideoTexture );

    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );

    glTexImage2D(GL_TEXTURE_2D,     // Type of texture
                    0,                 // Pyramid level (for mip-mapping) - 0 is the top level
                    GL_RGB,            // Internal colour format to convert to
                    m_nRenderWidth,          // Image width  i.e. 640 for Kinect in standard mode
                    m_nRenderHeight,          // Image height i.e. 480 for Kinect in standard mode
                    0,                 // Border width in pixels (can either be 1 or 0)
                    GL_BGR, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                    GL_UNSIGNED_BYTE,  // Image data type
                    defaultImage.ptr());        // The actual image data itself


    glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &fLargest);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, fLargest);

    glBindTexture( GL_TEXTURE_2D, 0 );


    return ( leftEyeVideoTexture != 0 && rightEyeVideoTexture != 0 );
}


//-----------------------------------------------------------------------------
// Purpose: create a sea of cubes
//-----------------------------------------------------------------------------
void RGBD2VR::SetupScene()
{
    if ( !m_pHMD )
        return;

    // Background: 2 triangles covering the whole screen
    {
        vector<float> bgVertices = {
            // x, y, z, u, v
            -1.0f, -1.0f, 0.f, 0.0f, 0.0f,
            1.0f, -1.0f, 0.f, 1.0f, 0.0f,
            -1.0f,  1.0f, 0.f, 0.0f,  1.0f,
            1.0f,  1.0f, 0.f, 1.0f,  1.0f
        };

        glGenVertexArrays( 1, &m_BackgroundVAO );
        glBindVertexArray( m_BackgroundVAO );

        glGenBuffers( 1, &m_glBackgroundVertBuffer );
        glBindBuffer( GL_ARRAY_BUFFER, m_glBackgroundVertBuffer );
        glBufferData( GL_ARRAY_BUFFER, sizeof(float) * bgVertices.size(), &bgVertices[0], GL_STATIC_DRAW);

        GLsizei stride = sizeof(VertexDataScene);
        uintptr_t offset = 0;

        glEnableVertexAttribArray( 0 );
        glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, stride , (const void *)offset);

        offset += sizeof(Vector3);
        glEnableVertexAttribArray( 1 );
        glVertexAttribPointer( 1, 2, GL_FLOAT, GL_FALSE, stride, (const void *)offset);

        glBindVertexArray( 0 );
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
    }

    // remaining of the scene (cubes)
    {
        std::vector<float> vertdataarray;

        Matrix4 matScale;
        matScale.scale( m_fScale, m_fScale, m_fScale );
        Matrix4 matTransform;
        matTransform.translate(
                -( (float)m_iSceneVolumeWidth * m_fScaleSpacing ) / 2.f,
                -( (float)m_iSceneVolumeHeight * m_fScaleSpacing ) / 2.f,
                -( (float)m_iSceneVolumeDepth * m_fScaleSpacing ) / 2.f);

        Matrix4 mat = matScale * matTransform;

        for( int z = 0; z< m_iSceneVolumeDepth; z++ )
        {
            for( int y = 0; y< m_iSceneVolumeHeight; y++ )
            {
                for( int x = 0; x< m_iSceneVolumeWidth; x++ )
                {
                    AddCubeToScene( mat, vertdataarray );
                    mat = mat * Matrix4().translate( m_fScaleSpacing, 0, 0 );
                }
                mat = mat * Matrix4().translate( -((float)m_iSceneVolumeWidth) * m_fScaleSpacing, m_fScaleSpacing, 0 );
            }
            mat = mat * Matrix4().translate( 0, -((float)m_iSceneVolumeHeight) * m_fScaleSpacing, m_fScaleSpacing );
        }
        m_uiVertcount = vertdataarray.size()/5;


        glGenVertexArrays( 1, &m_unSceneVAO );
        glBindVertexArray( m_unSceneVAO );

        glGenBuffers( 1, &m_glSceneVertBuffer );
        glBindBuffer( GL_ARRAY_BUFFER, m_glSceneVertBuffer );
        glBufferData( GL_ARRAY_BUFFER, sizeof(float) * vertdataarray.size(), &vertdataarray[0], GL_STATIC_DRAW);

        GLsizei stride = sizeof(VertexDataScene);
        uintptr_t offset = 0;

        glEnableVertexAttribArray( 0 );
        glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, stride , (const void *)offset);

        offset += sizeof(Vector3);
        glEnableVertexAttribArray( 1 );
        glVertexAttribPointer( 1, 2, GL_FLOAT, GL_FALSE, stride, (const void *)offset);

        glBindVertexArray( 0 );
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
    }
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void RGBD2VR::AddCubeVertex( float fl0, float fl1, float fl2, float fl3, float fl4, std::vector<float> &vertdata )
{
    vertdata.push_back( fl0 );
    vertdata.push_back( fl1 );
    vertdata.push_back( fl2 );
    vertdata.push_back( fl3 );
    vertdata.push_back( fl4 );
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void RGBD2VR::AddCubeToScene( Matrix4 mat, std::vector<float> &vertdata )
{
    // Matrix4 mat( outermat.data() );

    Vector4 A = mat * Vector4( 0, 0, 0, 1 );
    Vector4 B = mat * Vector4( 1, 0, 0, 1 );
    Vector4 C = mat * Vector4( 1, 1, 0, 1 );
    Vector4 D = mat * Vector4( 0, 1, 0, 1 );
    Vector4 E = mat * Vector4( 0, 0, 1, 1 );
    Vector4 F = mat * Vector4( 1, 0, 1, 1 );
    Vector4 G = mat * Vector4( 1, 1, 1, 1 );
    Vector4 H = mat * Vector4( 0, 1, 1, 1 );

    // triangles instead of quads
    AddCubeVertex( E.x, E.y, E.z, 0, 1, vertdata ); //Front
    AddCubeVertex( F.x, F.y, F.z, 1, 1, vertdata );
    AddCubeVertex( G.x, G.y, G.z, 1, 0, vertdata );
    AddCubeVertex( G.x, G.y, G.z, 1, 0, vertdata );
    AddCubeVertex( H.x, H.y, H.z, 0, 0, vertdata );
    AddCubeVertex( E.x, E.y, E.z, 0, 1, vertdata );

    AddCubeVertex( B.x, B.y, B.z, 0, 1, vertdata ); //Back
    AddCubeVertex( A.x, A.y, A.z, 1, 1, vertdata );
    AddCubeVertex( D.x, D.y, D.z, 1, 0, vertdata );
    AddCubeVertex( D.x, D.y, D.z, 1, 0, vertdata );
    AddCubeVertex( C.x, C.y, C.z, 0, 0, vertdata );
    AddCubeVertex( B.x, B.y, B.z, 0, 1, vertdata );

    AddCubeVertex( H.x, H.y, H.z, 0, 1, vertdata ); //Top
    AddCubeVertex( G.x, G.y, G.z, 1, 1, vertdata );
    AddCubeVertex( C.x, C.y, C.z, 1, 0, vertdata );
    AddCubeVertex( C.x, C.y, C.z, 1, 0, vertdata );
    AddCubeVertex( D.x, D.y, D.z, 0, 0, vertdata );
    AddCubeVertex( H.x, H.y, H.z, 0, 1, vertdata );

    AddCubeVertex( A.x, A.y, A.z, 0, 1, vertdata ); //Bottom
    AddCubeVertex( B.x, B.y, B.z, 1, 1, vertdata );
    AddCubeVertex( F.x, F.y, F.z, 1, 0, vertdata );
    AddCubeVertex( F.x, F.y, F.z, 1, 0, vertdata );
    AddCubeVertex( E.x, E.y, E.z, 0, 0, vertdata );
    AddCubeVertex( A.x, A.y, A.z, 0, 1, vertdata );

    AddCubeVertex( A.x, A.y, A.z, 0, 1, vertdata ); //Left
    AddCubeVertex( E.x, E.y, E.z, 1, 1, vertdata );
    AddCubeVertex( H.x, H.y, H.z, 1, 0, vertdata );
    AddCubeVertex( H.x, H.y, H.z, 1, 0, vertdata );
    AddCubeVertex( D.x, D.y, D.z, 0, 0, vertdata );
    AddCubeVertex( A.x, A.y, A.z, 0, 1, vertdata );

    AddCubeVertex( F.x, F.y, F.z, 0, 1, vertdata ); //Right
    AddCubeVertex( B.x, B.y, B.z, 1, 1, vertdata );
    AddCubeVertex( C.x, C.y, C.z, 1, 0, vertdata );
    AddCubeVertex( C.x, C.y, C.z, 1, 0, vertdata );
    AddCubeVertex( G.x, G.y, G.z, 0, 0, vertdata );
    AddCubeVertex( F.x, F.y, F.z, 0, 1, vertdata );
}


//-----------------------------------------------------------------------------
// Purpose: Draw all of the controllers as X/Y/Z lines
//-----------------------------------------------------------------------------
void RGBD2VR::RenderControllerAxes()
{
    // Don't attempt to update controllers if input is not available
    if( !m_pHMD->IsInputAvailable() )
        return;

    std::vector<float> vertdataarray;

    m_uiControllerVertcount = 0;
    m_iTrackedControllerCount = 0;

    for ( vr::TrackedDeviceIndex_t unTrackedDevice = vr::k_unTrackedDeviceIndex_Hmd + 1; unTrackedDevice < vr::k_unMaxTrackedDeviceCount; ++unTrackedDevice )
    {
        if ( !m_pHMD->IsTrackedDeviceConnected( unTrackedDevice ) )
            continue;

        if( m_pHMD->GetTrackedDeviceClass( unTrackedDevice ) != vr::TrackedDeviceClass_Controller )
            continue;

        m_iTrackedControllerCount += 1;

        if( !m_rTrackedDevicePose[ unTrackedDevice ].bPoseIsValid )
            continue;

        const Matrix4 & mat = m_rmat4DevicePose[unTrackedDevice];

        Vector4 center = mat * Vector4( 0, 0, 0, 1 );

        for ( int i = 0; i < 3; ++i )
        {
            Vector3 color( 0, 0, 0 );
            Vector4 point( 0, 0, 0, 1 );
            point[i] += 0.05f;  // offset in X, Y, Z
            color[i] = 1.0;  // R, G, B
            point = mat * point;
            vertdataarray.push_back( center.x );
            vertdataarray.push_back( center.y );
            vertdataarray.push_back( center.z );

            vertdataarray.push_back( color.x );
            vertdataarray.push_back( color.y );
            vertdataarray.push_back( color.z );

            vertdataarray.push_back( point.x );
            vertdataarray.push_back( point.y );
            vertdataarray.push_back( point.z );

            vertdataarray.push_back( color.x );
            vertdataarray.push_back( color.y );
            vertdataarray.push_back( color.z );

            m_uiControllerVertcount += 2;
        }

        Vector4 start = mat * Vector4( 0, 0, -0.02f, 1 );
        Vector4 end = mat * Vector4( 0, 0, -39.f, 1 );
        Vector3 color( .92f, .92f, .71f );

        vertdataarray.push_back( start.x );vertdataarray.push_back( start.y );vertdataarray.push_back( start.z );
        vertdataarray.push_back( color.x );vertdataarray.push_back( color.y );vertdataarray.push_back( color.z );

        vertdataarray.push_back( end.x );vertdataarray.push_back( end.y );vertdataarray.push_back( end.z );
        vertdataarray.push_back( color.x );vertdataarray.push_back( color.y );vertdataarray.push_back( color.z );
        m_uiControllerVertcount += 2;
    }

    // Setup the VAO the first time through.
    if ( m_unControllerVAO == 0 )
    {
        glGenVertexArrays( 1, &m_unControllerVAO );
        glBindVertexArray( m_unControllerVAO );

        glGenBuffers( 1, &m_glControllerVertBuffer );
        glBindBuffer( GL_ARRAY_BUFFER, m_glControllerVertBuffer );

        GLuint stride = 2 * 3 * sizeof( float );
        uintptr_t offset = 0;

        glEnableVertexAttribArray( 0 );
        glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, stride, (const void *)offset);

        offset += sizeof( Vector3 );
        glEnableVertexAttribArray( 1 );
        glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, stride, (const void *)offset);

        glBindVertexArray( 0 );
    }

    glBindBuffer( GL_ARRAY_BUFFER, m_glControllerVertBuffer );

    // set vertex data if we have some
    if( vertdataarray.size() > 0 )
    {
        //$ TODO: Use glBufferSubData for this...
        glBufferData( GL_ARRAY_BUFFER, sizeof(float) * vertdataarray.size(), &vertdataarray[0], GL_STREAM_DRAW );
    }
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void RGBD2VR::SetupCameras()
{
	m_mat4ProjectionLeft = GetHMDMatrixProjectionEye( vr::Eye_Left );
	m_mat4ProjectionRight = GetHMDMatrixProjectionEye( vr::Eye_Right );
	m_mat4eyePosLeft = GetHMDMatrixPoseEye( vr::Eye_Left );
	m_mat4eyePosRight = GetHMDMatrixPoseEye( vr::Eye_Right );
}


//-----------------------------------------------------------------------------
// Purpose: Creates a frame buffer. Returns true if the buffer was set up.
//          Returns false if the setup failed.
//-----------------------------------------------------------------------------
bool RGBD2VR::CreateFrameBuffer( int nWidth, int nHeight, FramebufferDesc &framebufferDesc )
{
	glGenFramebuffers(1, &framebufferDesc.m_nRenderFramebufferId );
	glBindFramebuffer(GL_FRAMEBUFFER, framebufferDesc.m_nRenderFramebufferId);

	glGenRenderbuffers(1, &framebufferDesc.m_nDepthBufferId);
	glBindRenderbuffer(GL_RENDERBUFFER, framebufferDesc.m_nDepthBufferId);
	glRenderbufferStorageMultisample(GL_RENDERBUFFER, 4, GL_DEPTH_COMPONENT, nWidth, nHeight );
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER,	framebufferDesc.m_nDepthBufferId );

	glGenTextures(1, &framebufferDesc.m_nRenderTextureId );
	glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, framebufferDesc.m_nRenderTextureId );
	glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, 4, GL_RGBA8, nWidth, nHeight, true);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, framebufferDesc.m_nRenderTextureId, 0);

	glGenFramebuffers(1, &framebufferDesc.m_nResolveFramebufferId );
	glBindFramebuffer(GL_FRAMEBUFFER, framebufferDesc.m_nResolveFramebufferId);

	glGenTextures(1, &framebufferDesc.m_nResolveTextureId );
	glBindTexture(GL_TEXTURE_2D, framebufferDesc.m_nResolveTextureId );
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, nWidth, nHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, framebufferDesc.m_nResolveTextureId, 0);

	// check FBO status
	GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (status != GL_FRAMEBUFFER_COMPLETE)
	{
		return false;
	}

	glBindFramebuffer( GL_FRAMEBUFFER, 0 );

	return true;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool RGBD2VR::SetupStereoRenderTargets()
{
	if ( !m_pHMD )
		return false;

	m_pHMD->GetRecommendedRenderTargetSize( &m_nRenderWidth, &m_nRenderHeight );

	CreateFrameBuffer( m_nRenderWidth, m_nRenderHeight, leftEyeDesc );
	CreateFrameBuffer( m_nRenderWidth, m_nRenderHeight, rightEyeDesc );
	
	return true;
}

void RGBD2VR::RenderBackground(vr::Hmd_Eye nEye ) {

    glUseProgram(m_BackgroundProgramID);

    glBindVertexArray( m_BackgroundVAO );
    if( nEye == vr::Eye_Left ) {
        glBindTexture( GL_TEXTURE_2D, leftEyeVideoTexture );
    }
    else {
        glBindTexture( GL_TEXTURE_2D, leftEyeVideoTexture );
        //glBindTexture( GL_TEXTURE_2D, rightEyeVideoTexture );
    }

    glDrawArrays( GL_TRIANGLE_STRIP, 0, 4 );
    glBindVertexArray( 0 );

    glUseProgram(0);

}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void RGBD2VR::RenderStereoTargets()
{
	glClearColor( 0.0f, 0.0f, 0.0f, 1.0f );
	glEnable( GL_MULTISAMPLE );

	// Left Eye
	glBindFramebuffer( GL_FRAMEBUFFER, leftEyeDesc.m_nRenderFramebufferId );
 	glViewport(0, 0, m_nRenderWidth, m_nRenderHeight );
 	RenderScene( vr::Eye_Left );
 	glBindFramebuffer( GL_FRAMEBUFFER, 0 );
	
	glDisable( GL_MULTISAMPLE );
	 	
 	glBindFramebuffer(GL_READ_FRAMEBUFFER, leftEyeDesc.m_nRenderFramebufferId);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, leftEyeDesc.m_nResolveFramebufferId );

    glBlitFramebuffer( 0, 0, m_nRenderWidth, m_nRenderHeight, 0, 0, m_nRenderWidth, m_nRenderHeight, 
		GL_COLOR_BUFFER_BIT,
 		GL_LINEAR );

 	glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0 );	

	glEnable( GL_MULTISAMPLE );

	// Right Eye
	glBindFramebuffer( GL_FRAMEBUFFER, rightEyeDesc.m_nRenderFramebufferId );
 	glViewport(0, 0, m_nRenderWidth, m_nRenderHeight );
 	RenderScene( vr::Eye_Right );
 	glBindFramebuffer( GL_FRAMEBUFFER, 0 );
 	
	glDisable( GL_MULTISAMPLE );

 	glBindFramebuffer(GL_READ_FRAMEBUFFER, rightEyeDesc.m_nRenderFramebufferId );
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, rightEyeDesc.m_nResolveFramebufferId );
	
    glBlitFramebuffer( 0, 0, m_nRenderWidth, m_nRenderHeight, 0, 0, m_nRenderWidth, m_nRenderHeight, 
		GL_COLOR_BUFFER_BIT,
 		GL_LINEAR  );

 	glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0 );
}

void RGBD2VR::RenderScene( vr::Hmd_Eye nEye )
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glDisable(GL_DEPTH_TEST);

	RenderBackground( vr::Eye_Right );

	glEnable(GL_DEPTH_TEST);

	if( m_bShowCubes )
	{
		glUseProgram( m_unSceneProgramID );
		glUniformMatrix4fv( m_nSceneMatrixLocation, 1, GL_FALSE, GetCurrentViewProjectionMatrix( nEye ).get() );
		glBindVertexArray( m_unSceneVAO );
		glBindTexture( GL_TEXTURE_2D, leftEyeVideoTexture );
		glDrawArrays( GL_TRIANGLES, 0, m_uiVertcount );
		glBindVertexArray( 0 );
	}

	bool bIsInputAvailable = m_pHMD->IsInputAvailable();

	if( bIsInputAvailable )
	{
		// draw the controller axis lines
		glUseProgram( m_unControllerTransformProgramID );
		glUniformMatrix4fv( m_nControllerMatrixLocation, 1, GL_FALSE, GetCurrentViewProjectionMatrix( nEye ).get() );
		glBindVertexArray( m_unControllerVAO );
		glDrawArrays( GL_LINES, 0, m_uiControllerVertcount );
		glBindVertexArray( 0 );
	}

	// ----- Render Model rendering -----
	glUseProgram( m_unRenderModelProgramID );

	for( uint32_t unTrackedDevice = 0; unTrackedDevice < vr::k_unMaxTrackedDeviceCount; unTrackedDevice++ )
	{
		if( !m_rTrackedDeviceToRenderModel[ unTrackedDevice ] || !m_rbShowTrackedDevice[ unTrackedDevice ] )
			continue;

		const vr::TrackedDevicePose_t & pose = m_rTrackedDevicePose[ unTrackedDevice ];
		if( !pose.bPoseIsValid )
			continue;

		if( !bIsInputAvailable && m_pHMD->GetTrackedDeviceClass( unTrackedDevice ) == vr::TrackedDeviceClass_Controller )
			continue;

		const Matrix4 & matDeviceToTracking = m_rmat4DevicePose[ unTrackedDevice ];
		Matrix4 matMVP = GetCurrentViewProjectionMatrix( nEye ) * matDeviceToTracking;
		glUniformMatrix4fv( m_nRenderModelMatrixLocation, 1, GL_FALSE, matMVP.get() );

		m_rTrackedDeviceToRenderModel[ unTrackedDevice ]->Draw();
	}

	glUseProgram( 0 );
}


//-----------------------------------------------------------------------------
// Purpose: Gets a Matrix Projection Eye with respect to nEye.
//-----------------------------------------------------------------------------
Matrix4 RGBD2VR::GetHMDMatrixProjectionEye( vr::Hmd_Eye nEye )
{
    if ( !m_pHMD )
        return Matrix4();

    vr::HmdMatrix44_t mat = m_pHMD->GetProjectionMatrix( nEye, m_fNearClip, m_fFarClip );

    return Matrix4(
            mat.m[0][0], mat.m[1][0], mat.m[2][0], mat.m[3][0],
            mat.m[0][1], mat.m[1][1], mat.m[2][1], mat.m[3][1], 
            mat.m[0][2], mat.m[1][2], mat.m[2][2], mat.m[3][2], 
            mat.m[0][3], mat.m[1][3], mat.m[2][3], mat.m[3][3]
            );
}


//-----------------------------------------------------------------------------
// Purpose: Gets an HMDMatrixPoseEye with respect to nEye.
//-----------------------------------------------------------------------------
Matrix4 RGBD2VR::GetHMDMatrixPoseEye( vr::Hmd_Eye nEye )
{
    if ( !m_pHMD )
        return Matrix4();

    vr::HmdMatrix34_t matEyeRight = m_pHMD->GetEyeToHeadTransform( nEye );
    Matrix4 matrixObj(
            matEyeRight.m[0][0], matEyeRight.m[1][0], matEyeRight.m[2][0], 0.0, 
            matEyeRight.m[0][1], matEyeRight.m[1][1], matEyeRight.m[2][1], 0.0,
            matEyeRight.m[0][2], matEyeRight.m[1][2], matEyeRight.m[2][2], 0.0,
            matEyeRight.m[0][3], matEyeRight.m[1][3], matEyeRight.m[2][3], 1.0f
            );

    return matrixObj.invert();
}


//-----------------------------------------------------------------------------
// Purpose: Gets a Current View Projection Matrix with respect to nEye,
//          which may be an Eye_Left or an Eye_Right.
//-----------------------------------------------------------------------------
Matrix4 RGBD2VR::GetCurrentViewProjectionMatrix( vr::Hmd_Eye nEye )
{
    Matrix4 matMVP;
    if( nEye == vr::Eye_Left )
    {
        matMVP = m_mat4ProjectionLeft * m_mat4eyePosLeft * m_mat4HMDPose;
    }
    else if( nEye == vr::Eye_Right )
    {
        matMVP = m_mat4ProjectionRight * m_mat4eyePosRight *  m_mat4HMDPose;
    }

    return matMVP;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void RGBD2VR::UpdateHMDMatrixPose()
{
    if ( !m_pHMD )
        return;

    vr::VRCompositor()->WaitGetPoses(m_rTrackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0 );

    m_iValidPoseCount = 0;
    m_strPoseClasses = "";
    for ( int nDevice = 0; nDevice < vr::k_unMaxTrackedDeviceCount; ++nDevice )
    {
        if ( m_rTrackedDevicePose[nDevice].bPoseIsValid )
        {
            m_iValidPoseCount++;
            m_rmat4DevicePose[nDevice] = ConvertSteamVRMatrixToMatrix4( m_rTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking );
            if (m_rDevClassChar[nDevice]==0)
            {
                switch (m_pHMD->GetTrackedDeviceClass(nDevice))
                {
                    case vr::TrackedDeviceClass_Controller:        m_rDevClassChar[nDevice] = 'C'; break;
                    case vr::TrackedDeviceClass_HMD:               m_rDevClassChar[nDevice] = 'H'; break;
                    case vr::TrackedDeviceClass_Invalid:           m_rDevClassChar[nDevice] = 'I'; break;
                    case vr::TrackedDeviceClass_GenericTracker:    m_rDevClassChar[nDevice] = 'G'; break;
                    case vr::TrackedDeviceClass_TrackingReference: m_rDevClassChar[nDevice] = 'T'; break;
                    default:                                       m_rDevClassChar[nDevice] = '?'; break;
                }
            }
            m_strPoseClasses += m_rDevClassChar[nDevice];
        }
    }

    if ( m_rTrackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid )
    {
        m_mat4HMDPose = m_rmat4DevicePose[vr::k_unTrackedDeviceIndex_Hmd];
        m_mat4HMDPose.invert();
    }
}


//-----------------------------------------------------------------------------
// Purpose: Finds a render model we've already loaded or loads a new one
//-----------------------------------------------------------------------------
CGLRenderModel *RGBD2VR::FindOrLoadRenderModel( const char *pchRenderModelName )
{
    CGLRenderModel *pRenderModel = NULL;
    for( std::vector< CGLRenderModel * >::iterator i = m_vecRenderModels.begin(); i != m_vecRenderModels.end(); i++ )
    {
        if( !strcmp( (*i)->GetName().c_str(), pchRenderModelName ) )
        {
            pRenderModel = *i;
            break;
        }
    }

    // load the model if we didn't find one
    if( !pRenderModel )
    {
        vr::RenderModel_t *pModel;
        vr::EVRRenderModelError error;
        while ( 1 )
        {
            error = vr::VRRenderModels()->LoadRenderModel_Async( pchRenderModelName, &pModel );
            if ( error != vr::VRRenderModelError_Loading )
                break;

            usleep( 1000 );
        }

        if ( error != vr::VRRenderModelError_None )
        {
            ROS_INFO( "Unable to load render model %s - %s\n", pchRenderModelName, vr::VRRenderModels()->GetRenderModelErrorNameFromEnum( error ) );
            return NULL; // move on to the next tracked device
        }

        vr::RenderModel_TextureMap_t *pTexture;
        while ( 1 )
        {
            error = vr::VRRenderModels()->LoadTexture_Async( pModel->diffuseTextureId, &pTexture );
            if ( error != vr::VRRenderModelError_Loading )
                break;

            usleep( 1000 );
        }

        if ( error != vr::VRRenderModelError_None )
        {
            ROS_INFO( "Unable to load render texture id:%d for render model %s\n", pModel->diffuseTextureId, pchRenderModelName );
            vr::VRRenderModels()->FreeRenderModel( pModel );
            return NULL; // move on to the next tracked device
        }

        pRenderModel = new CGLRenderModel( pchRenderModelName );
        if ( !pRenderModel->BInit( *pModel, *pTexture ) )
        {
            ROS_INFO( "Unable to create GL model from render model %s\n", pchRenderModelName );
            delete pRenderModel;
            pRenderModel = NULL;
        }
        else
        {
            m_vecRenderModels.push_back( pRenderModel );
        }
        vr::VRRenderModels()->FreeRenderModel( pModel );
        vr::VRRenderModels()->FreeTexture( pTexture );
    }
    return pRenderModel;
}


//-----------------------------------------------------------------------------
// Purpose: Create/destroy GL a Render Model for a single tracked device
//-----------------------------------------------------------------------------
void RGBD2VR::SetupRenderModelForTrackedDevice( vr::TrackedDeviceIndex_t unTrackedDeviceIndex )
{
    if( unTrackedDeviceIndex >= vr::k_unMaxTrackedDeviceCount )
        return;

    // try to find a model we've already set up
    std::string sRenderModelName = GetTrackedDeviceString( m_pHMD, unTrackedDeviceIndex, vr::Prop_RenderModelName_String );
    CGLRenderModel *pRenderModel = FindOrLoadRenderModel( sRenderModelName.c_str() );
    if( !pRenderModel )
    {
        std::string sTrackingSystemName = GetTrackedDeviceString( m_pHMD, unTrackedDeviceIndex, vr::Prop_TrackingSystemName_String );
        ROS_INFO( "Unable to load render model for tracked device %d (%s.%s)", unTrackedDeviceIndex, sTrackingSystemName.c_str(), sRenderModelName.c_str() );
    }
    else
    {
        m_rTrackedDeviceToRenderModel[ unTrackedDeviceIndex ] = pRenderModel;
        m_rbShowTrackedDevice[ unTrackedDeviceIndex ] = true;
    }
}


//-----------------------------------------------------------------------------
// Purpose: Create/destroy GL Render Models
//-----------------------------------------------------------------------------
void RGBD2VR::SetupRenderModels()
{
    memset( m_rTrackedDeviceToRenderModel, 0, sizeof( m_rTrackedDeviceToRenderModel ) );

    if( !m_pHMD )
        return;

    for( uint32_t unTrackedDevice = vr::k_unTrackedDeviceIndex_Hmd + 1; unTrackedDevice < vr::k_unMaxTrackedDeviceCount; unTrackedDevice++ )
    {
        if( !m_pHMD->IsTrackedDeviceConnected( unTrackedDevice ) )
            continue;

        SetupRenderModelForTrackedDevice( unTrackedDevice );
    }

}


//-----------------------------------------------------------------------------
// Purpose: Converts a SteamVR matrix to our local matrix class
//-----------------------------------------------------------------------------
Matrix4 RGBD2VR::ConvertSteamVRMatrixToMatrix4( const vr::HmdMatrix34_t &matPose )
{
    Matrix4 matrixObj(
            matPose.m[0][0], matPose.m[1][0], matPose.m[2][0], 0.0,
            matPose.m[0][1], matPose.m[1][1], matPose.m[2][1], 0.0,
            matPose.m[0][2], matPose.m[1][2], matPose.m[2][2], 0.0,
            matPose.m[0][3], matPose.m[1][3], matPose.m[2][3], 1.0f
            );
    return matrixObj;
}


//-----------------------------------------------------------------------------
// Purpose: Create/destroy GL Render Models
//-----------------------------------------------------------------------------
CGLRenderModel::CGLRenderModel( const std::string & sRenderModelName )
    : m_sModelName( sRenderModelName )
{
    m_glIndexBuffer = 0;
    m_glVertArray = 0;
    m_glVertBuffer = 0;
    m_glTexture = 0;
}


CGLRenderModel::~CGLRenderModel()
{
    Cleanup();
}


//-----------------------------------------------------------------------------
// Purpose: Allocates and populates the GL resources for a render model
//-----------------------------------------------------------------------------
bool CGLRenderModel::BInit( const vr::RenderModel_t & vrModel, const vr::RenderModel_TextureMap_t & vrDiffuseTexture )
{
    // create and bind a VAO to hold state for this model
    glGenVertexArrays( 1, &m_glVertArray );
    glBindVertexArray( m_glVertArray );

    // Populate a vertex buffer
    glGenBuffers( 1, &m_glVertBuffer );
    glBindBuffer( GL_ARRAY_BUFFER, m_glVertBuffer );
    glBufferData( GL_ARRAY_BUFFER, sizeof( vr::RenderModel_Vertex_t ) * vrModel.unVertexCount, vrModel.rVertexData, GL_STATIC_DRAW );

    // Identify the components in the vertex buffer
    glEnableVertexAttribArray( 0 );
    glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( vr::RenderModel_Vertex_t ), (void *)offsetof( vr::RenderModel_Vertex_t, vPosition ) );
    glEnableVertexAttribArray( 1 );
    glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, sizeof( vr::RenderModel_Vertex_t ), (void *)offsetof( vr::RenderModel_Vertex_t, vNormal ) );
    glEnableVertexAttribArray( 2 );
    glVertexAttribPointer( 2, 2, GL_FLOAT, GL_FALSE, sizeof( vr::RenderModel_Vertex_t ), (void *)offsetof( vr::RenderModel_Vertex_t, rfTextureCoord ) );

    // Create and populate the index buffer
    glGenBuffers( 1, &m_glIndexBuffer );
    glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, m_glIndexBuffer );
    glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( uint16_t ) * vrModel.unTriangleCount * 3, vrModel.rIndexData, GL_STATIC_DRAW );

    glBindVertexArray( 0 );

    // create and populate the texture
    glGenTextures(1, &m_glTexture );
    glBindTexture( GL_TEXTURE_2D, m_glTexture );

    glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, vrDiffuseTexture.unWidth, vrDiffuseTexture.unHeight,
            0, GL_RGBA, GL_UNSIGNED_BYTE, vrDiffuseTexture.rubTextureMapData );

    // If this renders black ask McJohn what's wrong.
    glGenerateMipmap(GL_TEXTURE_2D);

    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );

    GLfloat fLargest;
    glGetFloatv( GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &fLargest );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, fLargest );

    glBindTexture( GL_TEXTURE_2D, 0 );

    m_unVertexCount = vrModel.unTriangleCount * 3;

    return true;
}


//-----------------------------------------------------------------------------
// Purpose: Frees the GL resources for a render model
//-----------------------------------------------------------------------------
void CGLRenderModel::Cleanup()
{
    if( m_glVertBuffer )
    {
        glDeleteBuffers(1, &m_glIndexBuffer);
        glDeleteVertexArrays( 1, &m_glVertArray );
        glDeleteBuffers(1, &m_glVertBuffer);
        m_glIndexBuffer = 0;
        m_glVertArray = 0;
        m_glVertBuffer = 0;
    }
}


void CGLRenderModel::Draw()
{
    glBindVertexArray( m_glVertArray );

    glActiveTexture( GL_TEXTURE0 );
    glBindTexture( GL_TEXTURE_2D, m_glTexture );

    glDrawElements( GL_TRIANGLES, m_unVertexCount, GL_UNSIGNED_SHORT, 0 );

    glBindVertexArray( 0 );
}

