#include <SDL.h>
#include <GL/glew.h>
#include <SDL_opengl.h>
#include <GL/glu.h>
#include <string>
#include <cstdlib>

#include <opencv2/opencv.hpp>

#include <openvr.h>

#include "Matrices.h"


class CGLRenderModel
{
    public:
        CGLRenderModel( const std::string & sRenderModelName );
        ~CGLRenderModel();

        bool BInit( const vr::RenderModel_t & vrModel, const vr::RenderModel_TextureMap_t & vrDiffuseTexture );
        void Cleanup();
        void Draw();
        const std::string & GetName() const { return m_sModelName; }

    private:
        GLuint m_glVertBuffer;
        GLuint m_glIndexBuffer;
        GLuint m_glVertArray;
        GLuint m_glTexture;
        GLsizei m_unVertexCount;
        std::string m_sModelName;
};

//-----------------------------------------------------------------------------
// Purpose:
//------------------------------------------------------------------------------
class RGBD2VR
{
    public:
        RGBD2VR( int argc = 0, char *argv[] = {} );
        virtual ~RGBD2VR();

        bool BInit();
        bool BInitGL();
        bool BInitCompositor();

        void SetupRenderModels();

        void shutdown();

        void start();
        void stop();
        void step();

        bool HandleInput();
        bool shutdownRequested;
        void ProcessVREvent( const vr::VREvent_t & event );
        void RenderFrame();
        void setNextBackgroundFrames(cv::Mat rgb, cv::Mat depth);



        bool initializeTextures();

        void SetupScene();
        void AddCubeToScene( Matrix4 mat, std::vector<float> &vertdata );
        void AddCubeVertex( float fl0, float fl1, float fl2, float fl3, float fl4, std::vector<float> &vertdata );

        void RenderControllerAxes();

        bool SetupStereoRenderTargets();
        void SetupCameras();

        void RenderStereoTargets();
        void RenderBackground(vr::Hmd_Eye nEye );
        void RenderScene( vr::Hmd_Eye nEye );

        Matrix4 GetHMDMatrixProjectionEye( vr::Hmd_Eye nEye );
        Matrix4 GetHMDMatrixPoseEye( vr::Hmd_Eye nEye );
        Matrix4 GetCurrentViewProjectionMatrix( vr::Hmd_Eye nEye );
        void UpdateHMDMatrixPose();

        Matrix4 ConvertSteamVRMatrixToMatrix4( const vr::HmdMatrix34_t &matPose );

        GLuint CompileGLShader( const char *pchShaderName, const char *pchVertexShader, const char *pchFragmentShader );
        bool CreateAllShaders();

        void SetupRenderModelForTrackedDevice( vr::TrackedDeviceIndex_t unTrackedDeviceIndex );
        CGLRenderModel *FindOrLoadRenderModel( const char *pchRenderModelName );

    private: 
        bool m_bDebugOpenGL;
        bool m_bVerbose;
        bool m_bPerf;
        bool m_bVblank;
        bool m_bGlFinishHack;

        vr::IVRSystem *m_pHMD;
        vr::IVRRenderModels *m_pRenderModels;
        std::string m_strDriver;
        std::string m_strDisplay;
        vr::TrackedDevicePose_t m_rTrackedDevicePose[ vr::k_unMaxTrackedDeviceCount ];
        Matrix4 m_rmat4DevicePose[ vr::k_unMaxTrackedDeviceCount ];
        bool m_rbShowTrackedDevice[ vr::k_unMaxTrackedDeviceCount ];

    private: // SDL bookkeeping
        SDL_Window *m_pCompanionWindow;
        uint32_t m_nCompanionWindowWidth;
        uint32_t m_nCompanionWindowHeight;

        SDL_GLContext m_pContext;

    private: // OpenCV stuff
        float baseline_scale;
        cv::Mat frame;
        void fakeStereo(cv::InputArray rgb, cv::InputArray disparity, cv::OutputArray eye, vr::Hmd_Eye);
        cv::Mat resizeKeepAspectRatio(const cv::Mat &input, const cv::Size &dstSize);

    private: // OpenGL bookkeeping
        int m_iTrackedControllerCount;
        int m_iTrackedControllerCount_Last;
        int m_iValidPoseCount;
        int m_iValidPoseCount_Last;
        bool m_bShowCubes;

        std::string m_strPoseClasses;                            // what classes we saw poses for this frame
        char m_rDevClassChar[ vr::k_unMaxTrackedDeviceCount ];   // for each device, a character representing its class

        int m_iSceneVolumeWidth;
        int m_iSceneVolumeHeight;
        int m_iSceneVolumeDepth;
        float m_fScaleSpacing;
        float m_fScale;

        int m_iSceneVolumeInit;                                  // if you want something other than the default 20x20x20

        float m_fNearClip;
        float m_fFarClip;

        //GLuint m_iTexture;
        GLuint leftEyeVideoTexture;
        GLuint rightEyeVideoTexture;

        unsigned int m_uiVertcount;

        GLuint m_glBackgroundVertBuffer;
        GLuint m_BackgroundVAO;
        GLuint m_glSceneVertBuffer;
        GLuint m_unSceneVAO;
        GLuint m_unCompanionWindowVAO;
        GLuint m_glCompanionWindowIDVertBuffer;
        GLuint m_glCompanionWindowIDIndexBuffer;
        unsigned int m_uiCompanionWindowIndexSize;

        GLuint m_glControllerVertBuffer;
        GLuint m_unControllerVAO;
        unsigned int m_uiControllerVertcount;

        Matrix4 m_mat4HMDPose;
        Matrix4 m_mat4eyePosLeft;
        Matrix4 m_mat4eyePosRight;

        Matrix4 m_mat4ProjectionCenter;
        Matrix4 m_mat4ProjectionLeft;
        Matrix4 m_mat4ProjectionRight;

        struct VertexDataScene
        {
            Vector3 position;
            Vector2 texCoord;
        };

        struct VertexDataWindow
        {
            Vector2 position;
            Vector2 texCoord;

            VertexDataWindow( const Vector2 & pos, const Vector2 tex ) :  position(pos), texCoord(tex) {	}
        };

        GLuint m_BackgroundProgramID;
        GLuint m_unSceneProgramID;
        GLuint m_unCompanionWindowProgramID;
        GLuint m_unControllerTransformProgramID;
        GLuint m_unRenderModelProgramID;

        GLint m_nSceneMatrixLocation;
        GLint m_nControllerMatrixLocation;
        GLint m_nRenderModelMatrixLocation;

        struct FramebufferDesc
        {
            GLuint m_nDepthBufferId;
            GLuint m_nRenderTextureId;
            GLuint m_nRenderFramebufferId;
            GLuint m_nResolveTextureId;
            GLuint m_nResolveFramebufferId;
        };
        FramebufferDesc leftEyeDesc;
        FramebufferDesc rightEyeDesc;

        bool CreateFrameBuffer( int nWidth, int nHeight, FramebufferDesc &framebufferDesc );

        uint32_t m_nRenderWidth;
        uint32_t m_nRenderHeight;

        std::vector< CGLRenderModel * > m_vecRenderModels;
        CGLRenderModel *m_rTrackedDeviceToRenderModel[ vr::k_unMaxTrackedDeviceCount ];
};

