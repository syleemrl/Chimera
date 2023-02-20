#include <string>
#include <iostream>
#include <boost/program_options.hpp>
#include <GL/glut.h>
#include <QApplication>
#include <QGLWidget>
#include "SingleMotionMainWindow.h"
#include "ChimeraGenMainWindow.h"
#include "AssemblerMainWindow.h"
#include "RegressionMainWindow.h"
#include "RenderConfigParser.h"

class GLWidget : public QGLWidget{
    void initializeGL(){
        glClearColor(0.0, 1.0, 1.0, 1.0);
    }
    
    void qgluPerspective(GLdouble fovy, GLdouble aspect, GLdouble zNear, GLdouble zFar){
        const GLdouble ymax = zNear * tan(fovy * M_PI / 360.0);
        const GLdouble ymin = -ymax;
        const GLdouble xmin = ymin * aspect;
        const GLdouble xmax = ymax * aspect;
        glFrustum(xmin, xmax, ymin, ymax, zNear, zFar);
    }
    
    void resizeGL(int width, int height){
        if (height==0) height=1;
        glViewport(0,0,width,height);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        qgluPerspective(45.0f,(GLfloat)width/(GLfloat)height,0.1f,100.0f);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
     }
    
    void paintGL(){
        glMatrixMode(GL_MODELVIEW);         
        glLoadIdentity();
        glClear(GL_COLOR_BUFFER_BIT);  

        glBegin(GL_POLYGON); 
            glVertex2f(-0.5, -0.5); 
            glVertex2f(-0.5, 0.5);
            glVertex2f(0.5, 0.5); 
            glVertex2f(0.5, -0.5); 
        glEnd();
    }
};
int main(int argc,char** argv)
{
	boost::program_options::options_description desc("allowed options");
	desc.add_options()
	("config", boost::program_options::value<std::string>());


	boost::program_options::variables_map vm;
	boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
	std::string configuration = std::string(SSC_DIR) + std::string("/data/render/");


    if(vm.count("config")) {
        configuration += vm["config"].as<std::string>();
    }

    std::pair<std::string, std::vector<RenderData>> renderConfig = RenderConfigParser::parseRenderConfig(configuration);
    std::string type = renderConfig.first;

    std::cout << type << std::endl;
    RenderConfigParser::printRenderData(renderConfig.second);
	glutInit(&argc,argv);
	QApplication a(argc, argv);
    
    if(type == "single_motion") {
        SingleMotionMainWindow* main_window = new SingleMotionMainWindow(renderConfig.second);
        main_window->resize(2560,1440);
        main_window->show();
    } else if(type == "chimera_gen") {
        ChimeraGenMainWindow* main_window = new ChimeraGenMainWindow(renderConfig.second);
        main_window->resize(2560,1440);
        main_window->show();
    } else if(type == "assembler") {
        AssemblerMainWindow* main_window = new AssemblerMainWindow(renderConfig.second);
        main_window->resize(2560,1440);
        main_window->show();
    } else if(type == "regression") {
        RegressionMainWindow* main_window = new RegressionMainWindow(renderConfig.second);
        main_window->resize(2560,1440);
        main_window->show();
    } 
   
    return a.exec();
}
