#pragma once

#ifndef MY_WINDOW_HPP
#define MY_WINDOW_HPP


#include <QMainWindow>
#include <QString>




//forward declaration
namespace Ui
{
class MainWindow;
}
class myWidgetGL;

/** Declaration of the Window class */
class myWindow: public QMainWindow
{
    Q_OBJECT

public:

    myWindow(QWidget *parent=NULL);
    ~myWindow();



private slots:

    /** Quit the application */
    void action_quit();
    /** Enable the drawing of the meshes */
    void action_draw();
    /** Set the Wireframe mode for the meshes */
    void action_wireframe();

    void action_redraw();
    void action_confirm_bending(const float &k);
    void action_confirm_shearing(const float &k);
    void action_confirm_structural(const float &k);
    void action_toggle_wind();
    void action_wind_force_changed();
    void action_scale_changed();
    //void action_restart_simulation();
    void action_update_fps();

private:

    /** Layout for the Window */
    Ui::MainWindow *ui;
    /** The OpenGL Widget */
    myWidgetGL *glWidget;





};

#endif
