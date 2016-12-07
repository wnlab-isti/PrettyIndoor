#include <jni.h>
#include <string>

extern "C"
jstring
Java_it_cnr_isti_wnlab_indoornavigator_MainActivity_stringFromJNI(
        JNIEnv *env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}
