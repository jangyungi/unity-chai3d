#ifdef FUNCDLL_EXPORT
#define FUNCDLL_API __declspec(dllexport) 
#else
#define FUNCDLL_API __declspec(dllimport) 
#endif

extern "C" {
	FUNCDLL_API bool prepareHaptics(double hapticScale);
	FUNCDLL_API void startHaptics(void);
	FUNCDLL_API void stopHaptics(void);
	void updateHaptics(void);

	FUNCDLL_API void getProxyPosition(double outPosArray[]);
	FUNCDLL_API void getDevicePosition(double outPosArray[]);

	FUNCDLL_API bool isTouching(int objectId);
	FUNCDLL_API bool isButtonPressed(int buttonId);

	FUNCDLL_API void addObject(double objectPos[], double objectScale[], double objectRotation[], double vertPos[][3], double normals[][3], int vertNum, int tris[][3], int triNum);
	FUNCDLL_API void addModifiableObject(double objectPos[], double objectScale[], double objectRotation[], double vertPos[][3], double normals[][3], int vertNum, int triPos[][3], int triNum, double stiffness, double staticFriction, double dynamicFriction, double damping, double viscosity);

	FUNCDLL_API void translateObjects(double translation[]);
	FUNCDLL_API void setHapticPosition(double position[]);
	FUNCDLL_API void setHapticRotation(double rotation[]);

	FUNCDLL_API void testHaptics(void);
}

void convertXYZFromCHAI3D(double inputXYZ[]);
void convertXYZToCHAI3D(double inputXYZ[]);

