#include <com.badlogic.gdx.physics.box2d.ContactEdge.h>

#include <Box2D/Box2D.h>


JNIEXPORT jlong JNICALL Java_com_badlogic_gdx_physics_box2d_ContactEdge_jniGetNext
  (JNIEnv * env, jobject object, jlong addr){
	b2ContactEdge* contactEdge = (b2ContactEdge*)addr;
	return (jlong)contactEdge->next;
}

JNIEXPORT jlong JNICALL Java_com_badlogic_gdx_physics_box2d_ContactEdge_jniGetOther
  (JNIEnv * env, jobject object, jlong addr){
	b2ContactEdge* contactEdge = (b2ContactEdge*)addr;
	return (jlong)contactEdge->other;
}

JNIEXPORT jlong JNICALL Java_com_badlogic_gdx_physics_box2d_ContactEdge_jniGetContact
  (JNIEnv * env, jobject object, jlong addr){
	b2ContactEdge* contactEdge = (b2ContactEdge*)addr;
	return (jlong)contactEdge->contact;
}

