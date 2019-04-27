// The sample model.  You should build a file
// very similar to this for when you make your model.
#include "modelerview.h"
#include "modelerapp.h"
#include "modelerdraw.h"
#include "particleSystem.h"
#include "IK.h"
#include <FL/gl.h>
#include <math.h>

enum SampleModelControls
{
	XPOS, YPOS, ZPOS,  //whole body level transformations
	UPPER_BODY_ROTATE,
	LEFT_ARM_ROTATE, RIGHT_ARM_ROTATE,
	LEFT_FORE_ARM_ROTATE_X, LEFT_FORE_ARM_ROTATE_Y, RIGHT_FORE_ARM_ROTATE,
	HEAD_ROTATE, Level_Of_DETAILs, Torus, CrazyShoulders, ENABLEIK, IK_X, IK_Y, IK_Z, LEGCONSTRAINT, LSYSTEM, LSYSTEMLEVEL, Mood, Organic_Shape, Show_Texture, LIGHT_CONDITION,
	NUMCONTROLS
};

// Colors
#define COLOR_RED		1.0f, 0.0f, 0.0f
#define COLOR_GREEN		0.0f, 1.0f, 0.0f
#define COLOR_BLUE		0.0f, 0.0f, 1.0f
#define COLOR_PINK		1.0f, 0.5f, 0.25f
#define COLOR_VIOLET    0.63f, 0.125f, 0.94f
#define COLOR_GRAY      0.86f, 0.86f, 0.86f
#define COLOR_BLUE      0.1f, 0.1f, 0.44f
#define COLOR_GOLD      1.0f, 0.84f, 0.44f
#define VAL(x) (ModelerApplication::Instance()->GetControlValue(x))


// To make a SampleModel, we inherit off of ModelerView
class SampleModel : public ModelerView
{
public:
	SampleModel(int x, int y, int w, int h, char *label)
		: ModelerView(x, y, w, h, label) {
		Vec3f rightPoint2(-0.8, 3.2, 0);
		rightLeg = new InverseKinematics2(rightPoint2, 1.5, 1.7);
	}

	virtual void draw();
	void drawTorus();
	int additionAngel;
	//MetaBall* m_metaBall;
	void drawLower();
	void drawUpper();
	void drawThigh();
	void drawShank();
	void recursionTree3D(Vec3f dir, Vec3f nextdir, Vec3f currentLocation, float length);
	Vec3f* calculateNewDir(Vec3f newDir, Vec3f lastDir);
	void initRecurtionTree();
	float* getRotateAngles(Vec3f target);
	InverseKinematics2* rightLeg;
};

// We need to make a creator function, mostly because of
// nasty API stuff that we'd rather stay away from.
ModelerView* createSampleModel(int x, int y, int w, int h, char *label)
{
	return new SampleModel(x, y, w, h, label);
}

// We are going to override (is that the right word?) the draw()
// method of ModelerView to draw out SampleModel
void SampleModel::draw()
{

	Vec4f result1(0, 0, 0, 0);
	Vec4f result2(0, 0, 0, 0);
	// This call takes care of a lot of the nasty projection 
	// matrix stuff.  Unless you want to fudge directly with the 
	// projection matrix, don't bother with this ...
	ModelerView::draw();



	if (VAL(LIGHT_CONDITION)) {
		static GLfloat lightPosition2[] = { 1, 5, 2, 1 };
		static GLfloat lightDiffuse2[] = { 1, 1, 1, 1 };
		glEnable(GL_LIGHT2);
		glLightfv(GL_LIGHT2, GL_POSITION, lightPosition2);
		glLightfv(GL_LIGHT2, GL_DIFFUSE, lightDiffuse2);

	}
	else glDisable(GL_LIGHT2);


	if (VAL(ENABLEIK)) {
		Vec3f destination(VAL(IK_X), VAL(IK_Y), VAL(IK_Z));
		//cout << VAL(IKX) <<","<< VAL(IKY) << ","<< VAL(IKZ)<<endl;
		result2 = rightLeg->getResult(destination);
		glPushMatrix();
		setDiffuseColor(COLOR_BLUE);
		glTranslated(VAL(IK_X), VAL(IK_Y), VAL(IK_Z));
		drawBox(0.2, 0.2, 0.2);
		rightLeg->setConstraint((bool)VAL(LEGCONSTRAINT));
		//rightLeg->setConstraint1(VAL(LEGCONSTRAINT1));
		//rightLeg->setConstraint2(VAL(LEGCONSTRAINT2));

		glPopMatrix();
	}
	else
		rightLeg->reset();
	/*
	if (ModelerApplication::Instance()->m_animating == true) {
	additionAngel += 3;
	additionAngel % 360;
	}
	else {
	additionAngel = 0;
	}*/


	// draw the floor
	setAmbientColor(.1f, .1f, .1f);
	setDiffuseColor(COLOR_GRAY);
	glPushMatrix();
	glTranslated(-5, 0, -5);
	drawBox(10, 0.01f, 10);
	glPopMatrix();

	if (VAL(LSYSTEM)) {

		glPushMatrix();
		glTranslated(-3, 0, -3);
		initRecurtionTree();
		glPopMatrix();
	}

	// begin to draw the model
	setAmbientColor(.1f, .1f, .1f);
	setDiffuseColor(COLOR_GREEN);
	glPushMatrix();
	glTranslated(VAL(XPOS), VAL(YPOS), VAL(ZPOS));

	glPushMatrix(); // Upper body draw begin
	glRotated(VAL(UPPER_BODY_ROTATE) + additionAngel, 0.0, 1.0, 0.0);
	glPushMatrix(); // the chest draw begin
	setAmbientColor(.1f, .1f, .1f);
	setDiffuseColor(COLOR_GREEN);
	glTranslated(0, 5.5, 0);
	glRotated(90, 1, 0, 0);


	drawCylinder(1.5, 1, 1.5);

	glRotated(-90, 1, 0, 0);
	glTranslated(0, -5.5, 0);
	glPopMatrix(); //  chest draw ends

	if (VAL(Level_Of_DETAILs) > 1) {
		glPushMatrix(); // neck draw begin
		setAmbientColor(.1f, .1f, .1f);
		setDiffuseColor(COLOR_GREEN);
		glTranslated(0, 6, 0);
		glRotated(90, 1, 0, 0);
		drawCylinder(0.5, 0.2, 0.2);
		glRotated(-90, 1, 0, 0);
		glTranslated(0, -6, 0);
		glPopMatrix(); //  neck draw ends
	}
	if (VAL(Level_Of_DETAILs) > 2) {
		glPushMatrix(); // head draw begin
		glRotated(VAL(HEAD_ROTATE), 0.0, 1.0, 0.0);
		setAmbientColor(.1f, .1f, .1f); // tou draw begin
		setDiffuseColor(COLOR_VIOLET);
		glTranslated(0, 7, 0);
		glRotated(90, 1, 0, 0);
		drawSphere(1);
		glRotated(-90, 1, 0, 0);
		glTranslated(0, -7, 0);
		if (VAL(Level_Of_DETAILs) > 3) {

			setAmbientColor(.1f, .1f, .1f); // hat draw begin
			setDiffuseColor(COLOR_RED);
			glTranslated(-0.25, 8.5, -0.25);
			//glRotated(90, 1, 0, 0);

			drawRectangularPyramid(0.5, 0.7);

			if (VAL(Torus)) {
				setDiffuseColor(COLOR_GOLD);
				glTranslated(0.25, 0.7, 0.25);
				glScaled(0.1, 0.1, 0.1);
				glRotated(-90, 1, 0, 0);
				//drawTorus();
			}
			//glRotated(-90, 1, 0, 0);
			glTranslated(0.25, -8.5, 0.25);
		}
		glPopMatrix(); //  head draw ends
	}
	//drawCylinder(1, 0.3, 0.3);
	//drawSphere(0.4);

	glPushMatrix(); // left arm begin
	glTranslated(1.4, 5.5, 0);


	if (VAL(Mood) == 0)

		glRotated(VAL(LEFT_ARM_ROTATE), 0.0, 1.0, 0.0);

	else

		glRotated(VAL(LEFT_ARM_ROTATE), 0.0, 0.0, 1.0);
	glPushMatrix(); // first joint beigin
	setDiffuseColor(COLOR_RED);
	if (VAL(CrazyShoulders) == 0)
	{
		glRotated(90, 1, 0, 0);
		drawSphere(0.4);
		glRotated(-90, 1, 0, 0);
	}

	glPopMatrix();

	if (VAL(Level_Of_DETAILs) > 1) {
		setDiffuseColor(COLOR_PINK);
		glPushMatrix();
		glRotated(90, 1, 0, 0); // upper arm beigin
		glRotated(30, 0, 1, 0);
		drawCylinder(1, 0.3, 0.3);
		glRotated(-90, 1, 0, 0);
		glRotated(-30, 0, 1, 0);
		glPopMatrix();

		if (VAL(Level_Of_DETAILs) > 2) {
			glPushMatrix(); //lower arm begin
			glTranslated(0.6, -1.039, 0);
			//glTranslated(0.4, 0, 0);

			//glTranslated(-0.4, 0, 0);
			//glPushMatrix();
			glRotated(90, 1, 0, 0);
			drawSphere(0.4);
			glRotated(30, 0, 1, 0);

			glRotated(VAL(LEFT_FORE_ARM_ROTATE_X), 1.0, 0.0, 0.0);
			glRotated(VAL(LEFT_FORE_ARM_ROTATE_Y), 0.0, 1.0, 0.0);
			//glRotated(VAL(LEFT_FORE_ARM_ROTATE_Z), 0.0, 0.0, 1.0);

			drawCylinder(1, 0.3, 0.3);
			glRotated(-90, 1, 0, 0);
			glRotated(-30, 0, 1, 0);
			if (VAL(Level_Of_DETAILs) > 3) {
				glPushMatrix();
				setAmbientColor(.1f, .1f, .1f);
				setDiffuseColor(COLOR_VIOLET);
				glTranslated(0, -1.2, 0);
				glRotated(90, 1, 0, 0);
				drawSphere(0.4);
				glRotated(-90, 1, 0, 0);
				glPopMatrix();
			}
			//glPopMatrix();
			glPopMatrix();
		}
	}
	glTranslated(-1.4, -5.5, 0);
	glPopMatrix(); //  left arm ends



	glPushMatrix(); // right arm begin
	glTranslated(-1.4, 5.5, 0);
	if (VAL(Mood) == 0)

		glRotated(VAL(RIGHT_ARM_ROTATE), 0.0, 1.0, 0.0);
	else
		glRotated(VAL(RIGHT_ARM_ROTATE), 0.0, 0.0, 1.0);


	glPushMatrix(); // first joint beigin

	setAmbientColor(.1f, .1f, .1f);
	setDiffuseColor(COLOR_RED);

	//draw simple shoulder
	glRotated(90, 1, 0, 0);
	drawSphere(0.4);
	glRotated(-90, 1, 0, 0);
	/*
	if (VAL(CrazyShoulders) == 0)
	{
	glRotated(90, 1, 0, 0);
	drawSphere(0.4);
	glRotated(-90, 1, 0, 0);
	}
	if (VAL(CrazyShoulders) == 1)
	{
	glTranslated(-0.2, 0, -0.25);
	drawShoulder(0.5, 0.2);
	}*/
	glPopMatrix();

	if (VAL(Level_Of_DETAILs) > 1) {
		setDiffuseColor(COLOR_PINK);
		glPushMatrix();
		glRotated(90, 1, 0, 0); // upper arm beigin
		glRotated(-30, 0, 1, 0);
		drawCylinder(1, 0.3, 0.3);
		glRotated(-90, 1, 0, 0);
		glRotated(30, 0, 1, 0);
		glPopMatrix();

		if (VAL(Level_Of_DETAILs) > 2) {
			glPushMatrix(); //lower arm begin
			glTranslated(-0.6, -1.039, 0);
			glRotated(VAL(RIGHT_FORE_ARM_ROTATE), 1.0, 0.0, 0.0);
			glPushMatrix();
			glRotated(90, 1, 0, 0);
			drawSphere(0.4);
			glRotated(-30, 0, 1, 0);
			drawCylinder(1, 0.3, 0.3);
			glRotated(-90, 1, 0, 0);
			glRotated(30, 0, 1, 0);

			if (VAL(Level_Of_DETAILs) > 3) {
				glPushMatrix();
				setAmbientColor(.1f, .1f, .1f);
				setDiffuseColor(COLOR_VIOLET);
				glTranslated(0, -1.2, 0);
				glRotated(90, 1, 0, 0);
				drawSphere(0.4);
				glRotated(-90, 1, 0, 0);
				glPopMatrix();
			}
			glPopMatrix();
			glPopMatrix();
		}

	}
	glTranslated(1.4, -5.5, 0);
	glPopMatrix(); //  right arm ends



	glPopMatrix(); // upper body draw end


	glPushMatrix(); // lower body draw begin
					//glRotated(VAL(LOWER_BODY_ROTATE),0.0,1.0,0.0);

	glPushMatrix(); // the crotch draw begin
					//setAmbientColor(.1f, .1f, .1f);
					//if (VAL(Level_Of_DETAILs > 1)) {
	setDiffuseColor(COLOR_BLUE);
	glTranslated(0, 4, 0);
	glRotated(90, 1, 0, 0);
	//drawCylinder(1, 1.5, 1);
	//glRotated(-90, 1, 0, 0);
	//glTranslated(0, -4, 0);

	drawCylinder(1.5, 1.5, 1);

	glPopMatrix();

	setDiffuseColor(COLOR_PINK);
	glTranslated(0, 4.2, 0);
	glPushMatrix();
	glTranslated(-0.8, -1.0, 0);
	glPushMatrix();
	glRotated(result2[1], 0, 1, 0);
	glPushMatrix();
	glRotated(result2[0], 1, 0, 0);
	drawThigh();
	glPopMatrix();
	glPopMatrix();
	glPopMatrix();
	glPopMatrix();//pop translate m1

	glPushMatrix();
	glTranslated(rightLeg->joint[0], rightLeg->joint[1], rightLeg->joint[2]);
	glPushMatrix();
	glRotated(result2[3], 0, 1, 0);
	glPushMatrix();
	glRotated(result2[2], 1, 0, 0);
	drawShank();
	glPopMatrix();
	glPopMatrix();
	glPopMatrix();

	//left leg no ik
	glPushMatrix();
	setDiffuseColor(COLOR_PINK);
	glTranslated(0.8, 3.2, 0);
	drawThigh();
	glPushMatrix();
	glTranslated(0, -1.5, 0);
	drawShank();
	glPopMatrix();
	glPopMatrix();



	switch ((int)VAL(Mood)) {

	case 1:

		//ModelerApplication::Instance()->SetControlValue(LEFT_FORE_LEG_ROTATE, 90);

		//ModelerApplication::Instance()->SetControlValue(LEFT_UPPER_LEG_ROTATE, 72);

		//ModelerApplication::Instance()->SetControlValue(RIGHT_UPPER_LEG_ROTATE, -72);

		ModelerApplication::Instance()->SetControlValue(LEFT_ARM_ROTATE, 72);

		ModelerApplication::Instance()->SetControlValue(RIGHT_ARM_ROTATE, -72);
		break;
	case 0:
		//ModelerApplication::Instance()->SetControlValue(LEFT_FORE_LEG_ROTATE, 90);

		//ModelerApplication::Instance()->SetControlValue(LEFT_UPPER_LEG_ROTATE, 0);

		//ModelerApplication::Instance()->SetControlValue(RIGHT_UPPER_LEG_ROTATE, 0);
		ModelerApplication::Instance()->SetControlValue(LEFT_ARM_ROTATE, 0);
		ModelerApplication::Instance()->SetControlValue(RIGHT_ARM_ROTATE, 0);
		break;
	}
	glPopMatrix();

}


int main()
{
	// Initialize the controls
	// Constructor is ModelerControl(name, minimumvalue, maximumvalue, 
	// stepsize, defaultvalue)
	ModelerControl controls[NUMCONTROLS];
	controls[XPOS] = ModelerControl("X Position", -5, 5, 0.1f, 0);
	controls[YPOS] = ModelerControl("Y Position", 0, 5, 0.1f, 0);
	controls[ZPOS] = ModelerControl("Z Position", -5, 5, 0.1f, 0);
	//controls[HEIGHT] = ModelerControl("Height", 1, 2.5, 0.1f, 1);
	//controls[ROTATE] = ModelerControl("Rotate", -135, 135, 1, 0);
	controls[UPPER_BODY_ROTATE] = ModelerControl("Upper body rotate", -135, 135, 1, 0);
	//controls[LOWER_BODY_ROTATE] = ModelerControl("Lower body rotate", -135, 135, 1, 0);
	//controls[LEFT_ARM_ROTATE_X] = ModelerControl("Left arm rotateX", -135, 135, 1, 0);
	//controls[LEFT_ARM_ROTATE_Y] = ModelerControl("Left arm rotateY", -135, 135, 1, 0);
	controls[LEFT_ARM_ROTATE] = ModelerControl("Left arm rotateZ", -135, 135, 1, 0);
	controls[RIGHT_ARM_ROTATE] = ModelerControl("Right arm rotate", -135, 135, 1, 0);
	controls[LEFT_FORE_ARM_ROTATE_X] = ModelerControl("Left fore-arm rotateX", -135, 135, 1, 0);
	controls[LEFT_FORE_ARM_ROTATE_Y] = ModelerControl("Left fore-arm rotateY", -135, 135, 1, 0);
	//controls[LEFT_FORE_ARM_ROTATE_Z] = ModelerControl("Left fore-arm rotateZ", -135, 135, 1, 0);
	controls[RIGHT_FORE_ARM_ROTATE] = ModelerControl("Right fore-arm rotate", -135, 135, 1, 0);
	//controls[LEFT_FOOT_ROTATE_X] = ModelerControl("Left foot rotateX", -135, 135, 1, 0);
	//controls[RIGHT_FOOT_ROTATE_X] = ModelerControl("Right foot rotateX", -135, 135, 1, 0);
	//controls[LEFT_FOOT_ROTATE_Y] = ModelerControl("Left foot rotateY", -135, 135, 1, 0);
	//controls[RIGHT_FOOT_ROTATE_Y] = ModelerControl("Right foot rotateY", -135, 135, 1, 0);
	//controls[LEFT_FORE_LEG_ROTATE] = ModelerControl("Left fore-leg rotate", -135, 135, 1, 0);
	//controls[RIGHT_FORE_LEG_ROTATE] = ModelerControl("Right fore-leg rotate", -135, 135, 1, 0);
	//controls[LEFT_UPPER_LEG_ROTATE] = ModelerControl("Left Upper-leg rotate", -135, 135, 1, 0);
	//controls[RIGHT_UPPER_LEG_ROTATE] = ModelerControl("Right Upper-leg rotate", -135, 135, 1, 0);
	controls[HEAD_ROTATE] = ModelerControl("Head rotate", -135, 135, 1, 0);
	controls[Level_Of_DETAILs] = ModelerControl("Level of details", 1, 4, 1, 4);
	controls[Torus] = ModelerControl("Torus", 0, 1, 1, 0);
	controls[CrazyShoulders] = ModelerControl("CrazyShoulders", 0, 1, 1, 0);
	controls[ENABLEIK] = ModelerControl("IK", 0, 1, 1, 0);
	controls[IK_X] = ModelerControl("IK_X", -4.0, -0.8, 0.1, -0.8);
	controls[IK_Y] = ModelerControl("IK_Y", 0, 2.5, 0.1, 0);
	controls[IK_Z] = ModelerControl("IK_Z", -3.2, 2.8, 0.1, 0);
	controls[LEGCONSTRAINT] = ModelerControl("LEGCONSTRAINT", 0, 1, 1, 0);
	controls[LSYSTEM] = ModelerControl("Enable L-System", 0, 1, 1, 0);
	controls[LSYSTEMLEVEL] = ModelerControl("LSYSTEMLEVEL", 1, 5, 1, 3);
	controls[Mood] = ModelerControl("Mood", 0, 1, 1, 0);
	controls[Organic_Shape] = ModelerControl("Organic Shape", 0, 1, 1, 0);
	controls[Show_Texture] = ModelerControl("Texture", 0, 1, 1, 0);
	controls[LIGHT_CONDITION] = ModelerControl("Light Condition", 0, 1, 1, 0);


	ModelerApplication::Instance()->Init(&createSampleModel, controls, NUMCONTROLS);
	return ModelerApplication::Instance()->Run();
}

/*
void SampleModel::drawTorus() {
grid grid(30);

float c = 7;
float a = 3;
for (int i = 0; i < grid.numVertices; i++) {
float x = grid.vertices[i].position[0];
float y = grid.vertices[i].position[1];
float z = grid.vertices[i].position[2];

float s0 = c - sqrt(x * x + y * y);
if (s0 == 0 && z == 0)
z == 0.0001;

grid.vertices[i].value = (a * a) / (s0 * s0 + z * z);
//grid.vertices[i].value = (a * a) / (x * x + y * y + z * z);
//cout << grid.vertices[i].value << " ";
grid.vertices[i].normal[0] = grid.vertices[i].position[0];
grid.vertices[i].normal[1] = grid.vertices[i].position[1];
grid.vertices[i].normal[2] = grid.vertices[i].position[2];
grid.vertices[i].normal.normalize();
//if(grid.vertices[i].value > 1)
//cout << grid.vertices[i].value << " ";
}

glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
glEnable(GL_NORMALIZE);

grid.drawSurface(1.0);



}*/

/*
void SampleModel::drawLower() {
if (VAL(Level_Of_DETAILs) > 2) {
//left lower leg rotate
glTranslated(-0.7, 1.5, 0);
glRotated(-VAL(LEFT_FORE_LEG_ROTATE), 1.0, 0.0, 0.0);
glTranslated(0.7, -1.5, 0);

glPushMatrix();//left lower leg draw begin
glTranslated(-0.7, 1.5, 0);
glRotated(90, 1.0, 0.0, 0.0);
drawCylinder(1, 0.3, 0.3);
glPopMatrix();//left lower leg draw end
if (VAL(Level_Of_DETAILs) > 3) {
glPushMatrix();//left foot begin
setAmbientColor(.1f, .1f, .1f);
setDiffuseColor(COLOR_VIOLET);
glTranslated(-0.7, 0.5, 0.3);
glRotated(VAL(LEFT_FOOT_ROTATE_X), 1, 0, 0);
glRotated(VAL(LEFT_FOOT_ROTATE_Y), 0, 1, 0);
glScaled(1.5, 1, 2.5);
drawSphere(0.3);
glPopMatrix();//left foot end
}
}

}
void SampleModel::drawUpper() {
glPushMatrix();//left upper leg draw begin
glTranslated(-0.7, 2.5, 0);
glRotated(90, 1.0, 0.0, 0.0);
drawCylinder(1, 0.3, 0.3);
glPopMatrix();//left upper leg draw end

glPushMatrix();//left leg joint begin
glTranslated(-0.7, 1.5, 0);
drawSphere(0.4);
glPopMatrix();//left leg joint end

}
*/
void SampleModel::drawThigh() {


	glPushMatrix();
	glRotated(90, 1, 0, 0);
	drawCylinder(1.5, 0.25, 0.25);
	glTranslated(0, 0, 1.5);
	drawSphere(0.3);
	glPopMatrix();

}

void SampleModel::drawShank() {
	//glPushMatrix();
	//glTranslated(0, 3, 0);

	glPushMatrix();
	glRotated(90, 1, 0, 0);
	drawCylinder(1.5, 0.25, 0.25);


	glPushMatrix();
	setDiffuseColor(COLOR_VIOLET);
	//glTranslated(0, 0, 1.3);
	glTranslated(0, 0, 1.4);
	//glScaled(1, 1.7, 1.5);
	//drawCylinder(0.2, 0.45, 0.45);
	drawCylinder(0.3, 0.4, 0.4);
	//glRotated(90, 1, 0, 0);
	//drawSphere(0.3);
	glPopMatrix();
	glPopMatrix();

}

void SampleModel::recursionTree3D(Vec3f dir, Vec3f nextdir, Vec3f currentLocation, float length) {
	float width = length / 4.0f;
	nextdir *= length;
	if (length == 0.5) {

		glColor3ub(0, 255, 0);
		setAmbientColor(.1f, .1f, .1f);
		setDiffuseColor(COLOR_GREEN);
		//cout << nextdir[0] <<","<<nextdir[1]<<","<< nextdir[2] << endl;
		float* rotateAngle = getRotateAngles(nextdir);
		glPushMatrix();
		glTranslatef(currentLocation[0], currentLocation[1], currentLocation[2]);
		glPushMatrix();
		glRotatef(rotateAngle[1], 0, 1, 0);
		glPushMatrix();
		glRotatef(rotateAngle[0], 1, 0, 0);
		drawCylinder(length, 0.05, 0.05);
		glPopMatrix();
		glPopMatrix();
		glPopMatrix();
		return;
	}

	setAmbientColor(1.0f, 1.0f, 1.0f);
	//setDiffuseColor(0.545, 0.271, 0.075);
	setDiffuseColor(COLOR_RED);
	float* rotateAngle = getRotateAngles(nextdir);
	glPushMatrix();
	glTranslatef(currentLocation[0], currentLocation[1], currentLocation[2]);
	glPushMatrix();
	glRotatef(rotateAngle[1], 0, 1, 0);
	glPushMatrix();
	glRotatef(rotateAngle[0], 1, 0, 0);
	//cout << rotateAngle[0] << "," << rotateAngle[1] << endl;
	drawCylinder(length, 0.1, 0.1);
	glPopMatrix();
	glPopMatrix();
	glPopMatrix();
	currentLocation += nextdir;//update current location
	nextdir.normalize();
	Vec3f* newDir = calculateNewDir(dir, nextdir);
	//cout << newDir[0][0] << "," << newDir[0][1] << "," << newDir[0][2] << endl;
	recursionTree3D(nextdir, newDir[0], currentLocation, length / 2.0f);
	recursionTree3D(nextdir, newDir[1], currentLocation, length / 2.0f);
	recursionTree3D(nextdir, newDir[2], currentLocation, length / 2.0f);
	recursionTree3D(nextdir, newDir[3], currentLocation, length / 2.0f);
}

Vec3f* SampleModel::calculateNewDir(Vec3f newDir, Vec3f lastDir) {
	float deltaValue = newDir * lastDir;

	Vec3f lastDirNormal = lastDir;
	lastDirNormal.normalize();
	Vec3f projVec = deltaValue * lastDirNormal;
	Vec3f deltaVec = projVec - newDir;
	Vec3f deltaVecRotate90 = newDir ^ lastDir;
	deltaVecRotate90.normalize();
	deltaVecRotate90 *= deltaValue;
	Vec3f * result = new Vec3f[4];
	result[0] = projVec - deltaVec;
	result[2] = projVec + deltaVec;
	result[3] = projVec + deltaVecRotate90;
	result[1] = projVec - deltaVecRotate90;
	for (int i = 0; i < 4; i++)
		result[i].normalize();
	return result;
}

void SampleModel::initRecurtionTree() {
	Vec3f treeLocation(0, 0, 0);
	float trunkHeight = pow(2, VAL(LSYSTEMLEVEL) - 2);
	Vec3f inidirVec(0, 1, 0);
	Vec3f nextDirSeed(0, 1, 1);
	nextDirSeed.normalize();
	Vec3f* newDir = calculateNewDir(nextDirSeed, inidirVec);
	Vec3f currentLocation = inidirVec * trunkHeight;
	setAmbientColor(1.0f, 1.0f, 1.0f);
	setDiffuseColor(COLOR_RED);
	glPushMatrix();
	glRotated(-90, 1, 0, 0);
	drawCylinder(trunkHeight, 0.1, 0.1);
	glPopMatrix();
	recursionTree3D(inidirVec, newDir[0], currentLocation, trunkHeight);
	recursionTree3D(inidirVec, newDir[1], currentLocation, trunkHeight);
	recursionTree3D(inidirVec, newDir[2], currentLocation, trunkHeight);
	recursionTree3D(inidirVec, newDir[3], currentLocation, trunkHeight);
}



float* SampleModel::getRotateAngles(Vec3f target) {

	float* result = new float[2];
	Vec3f yaxis(0, 1, 0);
	Vec3f zaxis(0, 0, 1);
	Vec3f projVecY = target * yaxis * yaxis;
	Vec3f rotateVec = target - projVecY;
	Vec3f planeVec(0, 0, rotateVec.length());
	Vec3f planeTarget(0, projVecY.length(), rotateVec.length());
	result[0] = -acosf((zaxis * planeTarget) / planeTarget.length()) / 3.1416 * 180;
	if (rotateVec.length() != 0)
		result[1] = acosf((rotateVec * planeVec) / (rotateVec.length() * planeVec.length())) / 3.1416 * 180;
	else
		result[1] = 0;
	if (target[0] < 0)
		result[1] = -result[1];
	if (target[1] < 0)
		result[0] = -result[0];
	return result;
}