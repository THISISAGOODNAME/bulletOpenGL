//
// Created by AICDG on 2017/8/9.
//

#ifndef BULLETOPENGL_DEBUGDRAWER_H
#define BULLETOPENGL_DEBUGDRAWER_H

#include <LinearMath/btIDebugDraw.h>

class DebugDrawer : public btIDebugDraw {
public:
	// debug mode functions
	virtual void setDebugMode(int debugMode) override { m_debugMode = debugMode; }
	virtual int getDebugMode() const override { return m_debugMode; }

	// drawing functions
	virtual void  drawContactPoint(const btVector3 &pointOnB, const btVector3 &normalOnB, btScalar distance, int lifeTime, const btVector3 &color) override;
	virtual void  drawLine(const btVector3 &from, const btVector3 &to, const btVector3 &color) override;

	// unused
	virtual void  reportErrorWarning(const char* warningString) override {}
	virtual void  draw3dText(const btVector3 &location,const char* textString) override {}

	void ToggleDebugFlag(int flag);

protected:
	int m_debugMode;
};


#endif //BULLETOPENGL_DEBUGDRAWER_H
