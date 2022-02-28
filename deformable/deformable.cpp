// Include C++ headers
#include <iostream>
#include <string>
#include <map>
#include <fstream>
#include <string> 
#include <sstream>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <glfw3.h>

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

// Shader loading utilities and other
#include <common/shader.h>
#include <common/util.h>
#include <common/camera.h>
#include <common/model.h>
#include <common/texture.h>

// Extras
#include "Collision.h"
#include "RigidBody.h"
#include "Point-Spring-Handling.h"
#include "Grab.h"

using namespace std;
using namespace glm;

// Function prototypes
void initialize();
void createContext();
void mainLoop();
void free();
struct Light; struct Material;
void uploadMaterial(const Material& mtl);
void uploadLight(const Light& light);
void extractObjVertices(vector<RigidBody> rigids, vector<vec3>& vertices);
bool loadFileVertices(char* path, vector<vec3>& vertices);
void userMenu();
void handleMassKDamp(float& mass, float& k, float& damp, float dt);
void handleGrab(float dt);
void handleDistort(float dt);
void ffdCreateContext();
void ffdLoop();
void ffdExtractVertices(vector<RigidBody> rigids, vector<vec3>& vertices);
void ffdHandleGrab();
void ffdUpdate();
void handleNumbers();

#define W_WIDTH 1024
#define W_HEIGHT 768
#define TITLE "Deformable (kinda)"
// user model choices
#define CUBE '1'
#define SPHERE '2'
#define CYLINDER '3'
#define TEAPOT '4'
// user mode choices
#define BOUNCE '1'
#define GRAB '2'
#define DISTORT '3'
#define FFD '4' // Free Form Deformation

// global variables
GLFWwindow* window;
Camera* camera;
Grab* grab;
GLuint shaderProgram;
GLuint projectionMatrixLocation, viewMatrixLocation, modelMatrixLocation;
GLuint useTexture;
vector<int> objTriangles;
GLuint textureID, textureSampler;

// user choice variables
char userChoiceMode;
char userChoiceModel;
char userChoiceTexture;

// light properties
GLuint LaLocation, LdLocation, LsLocation, lightPositionLocation, lightPowerLocation;
// material properties
GLuint KdLocation, KsLocation, KaLocation, NsLocation;

// stairs variables
Drawable* stairsDraw;
vector<vec3> stairsVertices, stairsNormals;
vector<vec2> stairsUVs;

// model variables
Drawable* objDraw;
vector<RigidBody> objRigids;
vector<vector<float>> objPointRestingLengths;
vector<vec3> objVertices, objNormals;
vector<vec2> objUVs;
vector<vec3> vertexPositions;

// ffd model variables
float objEdges[3][2];
void findObjEdges();
vector<vec3> ffdVertexPositions;
vector<vec3> ffdInitialVertexPositions;
int vertexGrab;
Drawable* ffdTeaDraw;
vector<vec3> ffdTeaVertexPositions;
vector<vec3> ffdInitialTeaVertexPositions;
vector<vec3> ffdTeaVertices, ffdTeaNormals;
vector<vec2> ffdTeaUVs;
vector<vector<float>> ffdDefaultDistances;


struct Light {
	glm::vec4 La;
	glm::vec4 Ld;
	glm::vec4 Ls;
	glm::vec3 lightPosition_worldspace;
	float power;
};

struct Material {
	glm::vec4 Ka;
	glm::vec4 Kd;
	glm::vec4 Ks;
	float Ns;
};

Light light{
	vec4{ 1, 1, 1, 1 },
	vec4{ 1, 1, 1, 1 },
	vec4{ 1, 1, 1, 1 },
	vec3{ 2, 3, 3 },
	20.0f
};

Material goldMaterial{
	vec4{ 0.24725, 0.2245, 0.0645, 1 },
	vec4{ 0.34615, 0.3143, 0.0903, 1 },
	vec4{ 0.797357, 0.723991, 0.208006, 1 },
	83.2
};

Material stairMaterial{
	vec4{ 0.23125, 0.23125, 0.23125, 1 },
	vec4{ 0.2775, 0.2775, 0.2775, 1 },
	vec4{ 0.773911, 0.773911, 0.773911, 1 },
	89.6
};

void uploadMaterial(const Material& mtl) {
	glUniform4f(KaLocation, mtl.Ka.r, mtl.Ka.g, mtl.Ka.b, mtl.Ka.a);
	glUniform4f(KdLocation, mtl.Kd.r, mtl.Kd.g, mtl.Kd.b, mtl.Kd.a);
	glUniform4f(KsLocation, mtl.Ks.r, mtl.Ks.g, mtl.Ks.b, mtl.Ks.a);
	glUniform1f(NsLocation, mtl.Ns);
}

void uploadLight(const Light& light) {
	glUniform4f(LaLocation, light.La.r, light.La.g, light.La.b, light.La.a);
	glUniform4f(LdLocation, light.Ld.r, light.Ld.g, light.Ld.b, light.Ld.a);
	glUniform4f(LsLocation, light.Ls.r, light.Ls.g, light.Ls.b, light.Ls.a);
	glUniform3f(lightPositionLocation, light.lightPosition_worldspace.x,
		light.lightPosition_worldspace.y, light.lightPosition_worldspace.z);
	glUniform1f(lightPowerLocation, light.power);
}

void createContext() {

	// shader
	shaderProgram = loadShaders(
		"StandardShading.vertexshader",
		"StandardShading.fragmentshader");

	// get pointers to uniforms
	{
	modelMatrixLocation = glGetUniformLocation(shaderProgram, "M");
	viewMatrixLocation = glGetUniformLocation(shaderProgram, "V");
	projectionMatrixLocation = glGetUniformLocation(shaderProgram, "P");
	KaLocation = glGetUniformLocation(shaderProgram, "mtl.Ka");
	KdLocation = glGetUniformLocation(shaderProgram, "mtl.Kd");
	KsLocation = glGetUniformLocation(shaderProgram, "mtl.Ks");
	NsLocation = glGetUniformLocation(shaderProgram, "mtl.Ns");
	LaLocation = glGetUniformLocation(shaderProgram, "light.La");
	LdLocation = glGetUniformLocation(shaderProgram, "light.Ld");
	LsLocation = glGetUniformLocation(shaderProgram, "light.Ls");
	lightPositionLocation = glGetUniformLocation(shaderProgram, "light.lightPosition_worldspace");
	lightPowerLocation = glGetUniformLocation(shaderProgram, "light.power");
	textureSampler = glGetUniformLocation(shaderProgram, "TextureSampler");
	// Use Texture or not
	useTexture = glGetUniformLocation(shaderProgram, "useTexture");
	}

	// cube initialization
	if (userChoiceModel == CUBE)
	{
		loadFileVertices("models/cube.v5.obj", vertexPositions);
		// cube model loading
		loadOBJWithTiny("models/cube.v5.obj", objVertices, objUVs, objNormals);

		// cube texture
		if (userChoiceTexture == '1')
			textureID = loadBMP("textures/cubetexture.bmp");
		else
			textureID = loadBMP("textures/cubeWood.bmp");
	}

	// sphere initialization
	else if (userChoiceModel == SPHERE) {
		loadFileVertices("models/spherev2.obj", vertexPositions);
		// shpere model loading
		loadOBJWithTiny("models/spherev2.obj", objVertices, objUVs, objNormals);

		// sphere texture
		if (userChoiceTexture == '1')
			textureID = loadBMP("textures/footb.bmp");
		else
			textureID = loadBMP("textures/bball.bmp");
	}

	// cylinder initialization
	else if (userChoiceModel == CYLINDER) {
		loadFileVertices("models/cyl.obj", vertexPositions);
		// cylinder model loading
		loadOBJWithTiny("models/cyl.obj", objVertices, objUVs, objNormals);
	}

	// teapot initialization
	else if (userChoiceModel == TEAPOT) {
		loadFileVertices("models/tea.obj", vertexPositions);
		// teapot model loading
		loadOBJWithTiny("models/tea.obj", objVertices, objUVs, objNormals);
	}

	// create the drawable model
	objDraw = new Drawable(objVertices, objUVs, objNormals);

	// create a rigid body for every vertex
	for (int i = 0; i < vertexPositions.size(); i++) {
		objRigids.push_back(RigidBody());
		objRigids[i].x = vertexPositions[i];
		if (userChoiceMode == BOUNCE)
			objRigids[i].x -= vec3(1.0f, 0.0f, 0.0f);
	}

	// iterate over every distance between vertices and mark down their distance in resting position
	for (int i = 0; i < objRigids.size(); i++)
	{
		objPointRestingLengths.push_back(vector<float>());
		for (int j = 0; j < objRigids.size(); j++)
		{
			objPointRestingLengths[i].push_back(float());
			if (i == j)
				continue;
			objPointRestingLengths[i][j] = length(objRigids[i].x - objRigids[j].x);
		}
	}

	// stairs initialization
	{
		loadOBJWithTiny("models/stairs.obj", stairsVertices, stairsUVs, stairsNormals);
		stairsDraw = new Drawable(stairsVertices, stairsUVs, stairsNormals);
	}

	glUseProgram(shaderProgram);
}

void mainLoop() {
	camera->position = vec3(1.5, -1.0, 7.0);
	camera->update();
	if (userChoiceMode == BOUNCE)
		camera->position = vec3(-1.0, -0.5, 4.5);
	float mass = 1.0f;
	float dampFactor = 1.0f;
	float kFactor = 1.0f;
	float dt = 0.0035;
	if (userChoiceModel == CUBE) {
		dampFactor = 0.5f;
		dt = 0.00035f;
	}
	else if (userChoiceModel == SPHERE) {
		dt = 0.02f;
		dampFactor = 2.0f;
		mass = 2.0f;
	}
	else if (userChoiceModel == CYLINDER) {
		dt = 0.02f;
		mass = 2.0f;
	}
	else if (userChoiceModel == TEAPOT) {
		dt = 0.022f;
	}
	do {
		float time = glfwGetTime();
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Camera and Light
		{
			// camera
			if (userChoiceMode == GRAB || userChoiceMode == DISTORT);
			else
				camera->update();
			mat4 projectionMatrix = camera->projectionMatrix;
			mat4 viewMatrix = camera->viewMatrix;

			// light
			uploadLight(light);

			glUniformMatrix4fv(viewMatrixLocation, 1, GL_FALSE, &viewMatrix[0][0]);
			glUniformMatrix4fv(projectionMatrixLocation, 1, GL_FALSE, &projectionMatrix[0][0]);
			glUniformMatrix4fv(modelMatrixLocation, 1, GL_FALSE, &mat4()[0][0]);
		}

		// enable texture if it is supported
		if (userChoiceModel == CUBE || userChoiceModel == SPHERE) {
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, textureID);
			glUniform1i(useTexture, 1);
		}

		for (int i = 0; i < objRigids.size(); i++) {
			recalculatePointForces(objRigids, objPointRestingLengths, i, 1.0f, 1.0f);
			objRigids[i].advanceState(time, dt);
			checkStairCollision(objRigids[i]);
		}
		uploadMaterial(goldMaterial);
		extractObjVertices(objRigids, objVertices);
		objDraw->updateModel(objVertices, objUVs, objNormals);
		objDraw->bind();
		objDraw->draw();

		// Stairs
		{
			uploadMaterial(stairMaterial);
			stairsDraw->bind();
			glUniform1i(useTexture, 0);
			stairsDraw->draw();
		}

		
		float deltaTime = glfwGetTime() - time;

		if (userChoiceMode == GRAB) {
			grab->update();
			handleGrab(dt);
		}

		if (userChoiceMode == DISTORT) {
			grab->update();
			handleDistort(dt);
		}

		handleMassKDamp(mass, kFactor, dampFactor, deltaTime);

		glfwSwapBuffers(window);
		glfwPollEvents();
	} while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS && glfwWindowShouldClose(window) == 0);
}

void free() {
	glDeleteProgram(shaderProgram);
	glfwTerminate();
}

void extractObjVertices(vector<RigidBody> rigids, vector<vec3>& vertices) {
	for (int i = 0; i < objTriangles.size(); i++)
		vertices[i] = rigids[objTriangles[i]].x;
}

bool loadFileVertices(char* path, vector<vec3>& vertices) {
	FILE* file = fopen(path, "r");
	objTriangles.clear();
	if (file == NULL) {
		printf("Impossible to open the file !\n");
		return false;
	}
	while (1) {
		char lineHeader[128];
		// read the first word of the line
		int res = fscanf(file, "%s", lineHeader);
		if (res == EOF)
			break;
		if (strcmp(lineHeader, "v") == 0) {
			glm::vec3 vertex;
			int b = fscanf(file, "%f %f %f\n", &vertex.x, &vertex.y, &vertex.z);
			vertices.push_back(vertex);
		}
		else if (strcmp(lineHeader, "f") == 0) {
			std::string vertex1, vertex2, vertex3;
			unsigned int vertexIndex[3], uvIndex[3], normalIndex[3];
			int matches = fscanf(file, "%d/%d/%d %d/%d/%d %d/%d/%d\n", &vertexIndex[0], &uvIndex[0], &normalIndex[0], &vertexIndex[1], &uvIndex[1], &normalIndex[1], &vertexIndex[2], &uvIndex[2], &normalIndex[2]);
			if (matches != 9) {
				printf("File can't be read by our simple parser\n");
				return false;
			}
			objTriangles.push_back(vertexIndex[0] - 1);
			objTriangles.push_back(vertexIndex[1] - 1);
			objTriangles.push_back(vertexIndex[2] - 1);
		}
	}
	fclose(file);
	return true;
}

void initialize() {
	// Initialize GLFW
	if (!glfwInit()) {
		throw runtime_error("Failed to initialize GLFW\n");
	}

	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Open a window and create its OpenGL context
	window = glfwCreateWindow(W_WIDTH, W_HEIGHT, TITLE, NULL, NULL);
	if (window == NULL) {
		glfwTerminate();
		throw runtime_error(string(string("Failed to open GLFW window.") +
			" If you have an Intel GPU, they are not 3.3 compatible." +
			"Try the 2.1 version.\n"));
	}
	glfwMakeContextCurrent(window);

	// Start GLEW extension handler
	glewExperimental = GL_TRUE;

	// Initialize GLEW
	if (glewInit() != GLEW_OK) {
		glfwTerminate();
		throw runtime_error("Failed to initialize GLEW\n");
	}

	// Ensure we can capture the escape key being pressed below
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

	if (userChoiceMode == GRAB || userChoiceMode == DISTORT || userChoiceMode == FFD)
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
	else
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	// Set the mouse at the center of the screen
	glfwPollEvents();
	glfwSetCursorPos(window, W_WIDTH / 2, W_HEIGHT / 2);

	// Gray background color
	glClearColor(0.5f, 0.5f, 0.5f, 0.0f);

	// Enable depth test
	glEnable(GL_DEPTH_TEST);
	// Accept fragment if it closer to the camera than the former one
	glDepthFunc(GL_LESS);

	// Cull triangles which normal is not towards the camera
	 //glEnable(GL_CULL_FACE);

	// enable point size when drawing points
	glEnable(GL_PROGRAM_POINT_SIZE);

	// Log
	logGLParameters();

	// Create camera
	camera = new Camera(window);
	grab = new Grab(window);
}

void handleMassKDamp(float &mass, float& k, float& damp, float dt) {
	float speed = 3.0f;
	bool pressed = false;
	if (glfwGetKey(window, GLFW_KEY_U) == GLFW_PRESS) {
		pressed = true;
		mass += dt * speed;
		if (mass > 10.0f)
			mass = 10.0f;
		if (userChoiceModel == CUBE)
			for (int i = 0; i < objRigids.size(); i++)
				objRigids[i].m = mass;
		if (userChoiceModel == SPHERE)
			for (int i = 0; i < objRigids.size(); i++)
				objRigids[i].m = mass;
		if (userChoiceModel == CYLINDER)
			for (int i = 0; i < objRigids.size(); i++)
				objRigids[i].m = mass;
	}
	if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS) {
		pressed = true;
		mass -= dt * speed;
		if (mass < 0.5f)
			mass = 0.5f;
		if (userChoiceModel == CUBE)
			for (int i = 0; i < objRigids.size(); i++)
				objRigids[i].m = mass;
		if (userChoiceModel == SPHERE)
			for (int i = 0; i < objRigids.size(); i++)
				objRigids[i].m = mass;
		if (userChoiceModel == CYLINDER)
			for (int i = 0; i < objRigids.size(); i++)
				objRigids[i].m = mass;
	}

	if (glfwGetKey(window, GLFW_KEY_Y) == GLFW_PRESS) {
		pressed = true;
		k += dt * speed;
		if (k > 5.0f)
			k = 5.0f;
	}
	if (glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS) {
		pressed = true;
		k -= dt * speed;
		if (k < 0.5f)
			k = 0.5f;
	}

	if (glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS) {
		pressed = true;
		damp += dt * speed;
		if (damp > 5.0f)
			damp = 5.0f;
	}
	if (glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS) {
		pressed = true;
		damp -= dt * speed;
		if (damp < 0.5f)
			damp = 0.5f;
	}
	if (pressed)
	{
		cout << "\nMass: " << mass << " K-Factor: " << k << " Dampening Factor: " << damp;
	}
}

void handleGrab(float dt) {
	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
		if (userChoiceModel != CUBE)
			dt /= 10;
		float x = - grab->horizontalOffset * 1/dt * 1 / 1000;
		float y = grab->verticalOffset * 1/dt * 1 / 1000;
		for (int i = 0; i < objRigids.size(); i++) {
			objRigids[i].x.x += x;
			objRigids[i].x.y += y;
			objRigids[i].v.x = x;
			objRigids[i].v.y = y;
			objRigids[i].P.x = objRigids[i].v.x * objRigids[i].m * 1 / dt * 1 / 10;
			objRigids[i].P.y = objRigids[i].v.y * objRigids[i].m * 1 / dt * 1/10;
		}
	}
}

void handleDistort(float dt) {
	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
		if (userChoiceModel != CUBE)
			dt /= 10;
		float x = -grab->horizontalOffset * 1 / dt * 1 / 1000;
		float y = grab->verticalOffset * 1 / dt * 1 / 1000;
		cout << x << "\n";
		for (int j = 0; j < 3; j++) {
			int i = objTriangles[j];
			objRigids[i].x.x += x;
			objRigids[i].x.y += y;
			objRigids[i].v.x = x;
			objRigids[i].v.y = y;
			objRigids[i].P.x = objRigids[i].v.x * objRigids[i].m * 1 / dt * 1 / 10;
			objRigids[i].P.y = objRigids[i].v.y * objRigids[i].m * 1 / dt * 1 / 10;
		}
	}
}

//ffd
void ffdCreateContext() {
	// shader
	shaderProgram = loadShaders(
		"StandardShading.vertexshader",
		"StandardShading.fragmentshader");

	// get pointers to uniforms
	{
		modelMatrixLocation = glGetUniformLocation(shaderProgram, "M");
		viewMatrixLocation = glGetUniformLocation(shaderProgram, "V");
		projectionMatrixLocation = glGetUniformLocation(shaderProgram, "P");
		KaLocation = glGetUniformLocation(shaderProgram, "mtl.Ka");
		KdLocation = glGetUniformLocation(shaderProgram, "mtl.Kd");
		KsLocation = glGetUniformLocation(shaderProgram, "mtl.Ks");
		NsLocation = glGetUniformLocation(shaderProgram, "mtl.Ns");
		LaLocation = glGetUniformLocation(shaderProgram, "light.La");
		LdLocation = glGetUniformLocation(shaderProgram, "light.Ld");
		LsLocation = glGetUniformLocation(shaderProgram, "light.Ls");
		lightPositionLocation = glGetUniformLocation(shaderProgram, "light.lightPosition_worldspace");
		lightPowerLocation = glGetUniformLocation(shaderProgram, "light.power");
		textureSampler = glGetUniformLocation(shaderProgram, "TextureSampler");
		// Use Texture or not
		useTexture = glGetUniformLocation(shaderProgram, "useTexture");
	}

	loadFileVertices("models/tea_ffd.obj", vertexPositions);
	loadFileVertices("models/tea_ffd.obj", ffdInitialVertexPositions);

	// cube model loading
	//loadOBJWithTiny("models/tea_ffd.obj", vertexPositions);

	// create the drawable model
	objDraw = new Drawable(vertexPositions);

	// create a rigid body for every vertex
	for (int i = 0; i < vertexPositions.size(); i++) {
		objRigids.push_back(RigidBody());
		objRigids[i].x = vertexPositions[i];
		objRigids[i].x -= vec3(0.00001f, 0.00001f, 0.00001f);
	}

	// iterate over every distance between vertices and mark down their distance in resting position
	for (int i = 0; i < objRigids.size(); i++)
	{
		objPointRestingLengths.push_back(vector<float>());
		for (int j = 0; j < objRigids.size(); j++)
		{
			objPointRestingLengths[i].push_back(float());
			if (i == j)
				continue;
			objPointRestingLengths[i][j] = length(objRigids[i].x - objRigids[j].x);
		}
	}

	loadFileVertices("models/tea.obj", ffdInitialTeaVertexPositions);
	loadFileVertices("models/tea.obj", ffdTeaVertexPositions);
	// teapot model loading
	loadOBJWithTiny("models/tea.obj", ffdTeaVertices, ffdTeaUVs, ffdTeaNormals);
	ffdTeaDraw = new Drawable(ffdTeaVertices, ffdTeaUVs, ffdTeaNormals);

	for (int i = 0; i < ffdTeaVertexPositions.size(); i++) {
		ffdDefaultDistances.push_back(vector<float>());
		for (int j = 0; j < vertexPositions.size(); j++) {
			ffdDefaultDistances[i].push_back(length(ffdTeaVertexPositions[i] - vertexPositions[j]));
		}
	}
	vertexGrab = 0;
	glUseProgram(shaderProgram);
}

void ffdLoop() {
	camera->position = vec3(1.5, -1.0, 7.0);
	camera->update();
	float dt = 0.00035f;
	do
	{
		float time = glfwGetTime();
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) != GLFW_PRESS)
			camera->update();

		// Camera and Light
		{
			// camera
			mat4 projectionMatrix = camera->projectionMatrix;
			mat4 viewMatrix = camera->viewMatrix;

			// light
			uploadLight(light);

			glUniformMatrix4fv(viewMatrixLocation, 1, GL_FALSE, &viewMatrix[0][0]);
			glUniformMatrix4fv(projectionMatrixLocation, 1, GL_FALSE, &projectionMatrix[0][0]);
			glUniformMatrix4fv(modelMatrixLocation, 1, GL_FALSE, &mat4()[0][0]);
		}

		for (int i = 0; i < objRigids.size(); i++) {
			ffdRecalculatePointForces(objRigids, ffdInitialVertexPositions, i, 15.0f, 3.0f);
			objRigids[i].advanceState(time, dt);
		}
		uploadMaterial(goldMaterial);
		ffdExtractVertices(objRigids, vertexPositions);
		objDraw->updateModel(vertexPositions);
		objDraw->bind();
		objDraw->draw(GL_POINTS);

		ffdUpdate();

		uploadMaterial(stairMaterial);
		//extractObjVertices(objRigids, objVertices);
		for (int i = 0; i < objTriangles.size(); i++)
			ffdTeaVertices[i] = ffdTeaVertexPositions[objTriangles[i]];
		ffdTeaDraw->updateModel(ffdTeaVertices, ffdTeaUVs, ffdTeaNormals);
		glUniform1i(useTexture, 0);
		ffdTeaDraw->bind();
		ffdTeaDraw->draw();

		grab->update();
		ffdHandleGrab();
		handleNumbers();

		glfwSwapBuffers(window);
		glfwPollEvents();
	} while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS && glfwWindowShouldClose(window) == 0);
}

void findObjEdges() {
	loadFileVertices("models/tea.obj", vertexPositions);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 2; j++)
			objEdges[i][j] = vertexPositions[0][i];

	for (int indx = 1; indx < vertexPositions.size(); indx++)
	{
		for (int i = 0; i < 3; i++)
		{
			if (vertexPositions[indx][i] < objEdges[i][0])
				objEdges[i][0] = vertexPositions[indx][i];
			else if (vertexPositions[indx][i] > objEdges[i][1])
				objEdges[i][1] = vertexPositions[indx][i];
		}
	}
}

void ffdExtractVertices(vector<RigidBody> rigids, vector<vec3>& vertices) {
	for (int i = 0; i < vertices.size(); i++)
		vertices[i] = rigids[i].x;
}

void ffdHandleGrab() {
	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
		float dt = 0.0035f;
		float x = -grab->horizontalOffset * dt * 1000;
		float y = grab->verticalOffset * dt * 1000;
		objRigids[vertexGrab].x.x += x;
		objRigids[vertexGrab].x.y += y;
	}
}

void ffdUpdate() {
	for (int i = 0; i < ffdTeaVertexPositions.size(); i++)
	{
		ffdTeaVertexPositions[i] = ffdInitialTeaVertexPositions[i];
		for (int j = 0; j < vertexPositions.size(); j++)
		{
			vec3 dist = vertexPositions[j] - ffdInitialVertexPositions[j];
			vec3 direction = normalize(dist);
			float len = length(dist);
			vec3 dx = dist * (1 / ffdDefaultDistances[i][j]) / 5.0f;
			ffdTeaVertexPositions[i] += dx;
		}
	}
}

void handleNumbers() {
	if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS) {
		vertexGrab = 0;
	}
	if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS) {
		vertexGrab = 1;
	}
	if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS) {
		vertexGrab = 2;
	}
	if (glfwGetKey(window, GLFW_KEY_4) == GLFW_PRESS) {
		vertexGrab = 3;
	}
	if (glfwGetKey(window, GLFW_KEY_5) == GLFW_PRESS) {
		vertexGrab = 4;
	}
	if (glfwGetKey(window, GLFW_KEY_6) == GLFW_PRESS) {
		vertexGrab = 5;
	}
	if (glfwGetKey(window, GLFW_KEY_7) == GLFW_PRESS) {
		vertexGrab = 6;
	}
	if (glfwGetKey(window, GLFW_KEY_8) == GLFW_PRESS) {
		vertexGrab = 7;
	}
}

void userMenu() {
	cout << "Choose mode to demonstrate:" << endl;
	cout << "1. Bounce" << endl;
	cout << "2. Grab" << endl;
	cout << "3. Distort" << endl;
	cout << "4. Free Form Deformation" << endl;
	cout << "5. Plain fall" << endl;
	cin >> userChoiceMode;
	if (userChoiceMode == FFD)
		return;
	if (userChoiceMode == BOUNCE)
		userChoiceModel = CUBE;
	else
	{
		cout << "Choose model:" << endl;
		cout << "1. Cube" << endl;
		cout << "2. Sphere" << endl;
		cout << "3. Cylinder" << endl;
		cout << "4. Teapot" << endl;
		cin >> userChoiceModel;
	}

	if (userChoiceModel == CYLINDER || userChoiceModel == TEAPOT)
		return;
	cout << "Choose texture:" << endl;
	if (userChoiceModel == CUBE)
	{
		cout << "1. UoP" << endl;
		cout << "2. Wood" << endl;
	}
	else if (userChoiceModel == SPHERE)
	{
		cout << "1. Football" << endl;
		cout << "2. Basketball" << endl;
	}
	cin >> userChoiceTexture;
}

int main(void) {
	try {
		userMenu();
		initialize();
		if (userChoiceMode == FFD)
		{
			ffdCreateContext();
			ffdLoop();
		}
		else
		{
			createContext();
			mainLoop();
			free();
		}
	}
	catch (exception& ex) {
		cout << ex.what() << endl;
		int c = getchar();
		free();
		return -1;
	}

	return 0;
}