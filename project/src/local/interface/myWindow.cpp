#include "myWindow.hpp"

#include "myWidgetGL.hpp"
#include "../../lib/common/error_handling.hpp"
#include "ui_mainwindow.h"

#include <iostream>


myWindow::myWindow(QWidget *parent)
    :QMainWindow(parent),ui(new Ui::MainWindow)
{
    try
    {
        //Setup window layout
        ui->setupUi(this);

        //Create openGL context
        QGLFormat qglFormat;
        qglFormat.setVersion(1,2);

        //Create OpenGL Widget renderer
        glWidget=new myWidgetGL(qglFormat);

        //Add the OpenGL Widget into the layout
        ui->layout_scene->addWidget(glWidget);
        ui->k_bending_edit->setText(QString(glWidget->get_scene().get_mesh_cloth().str_k_bend().c_str()));
        ui->k_shearing_edit->setText(QString(glWidget->get_scene().get_mesh_cloth().str_k_shear().c_str()));
        ui->k_structural_edit->setText(QString(glWidget->get_scene().get_mesh_cloth().str_k_struct().c_str()));
        ui->toggle_wind->setChecked(glWidget->get_scene().get_wind());
        ui->wind_force_slider->setTracking(true);
        ui->wind_force_slider->setTickInterval(1);
        ui->wind_force_slider->setTickPosition(QSlider::TicksBelow);
        ui->wind_force_desc->setText(QString("wind force : ").append(QString::number(ui->wind_force_slider->value())));
    }
    catch(cpe::exception_cpe const& e)
    {
        std::cout<<std::endl<<e.report_exception()<<std::endl;
    }

    //Connect slot and signals
    connect(ui->quit,SIGNAL(clicked()),this,SLOT(action_quit()));
    connect(ui->draw,SIGNAL(clicked()),this,SLOT(action_draw()));
    connect(ui->wireframe,SIGNAL(clicked()),this,SLOT(action_wireframe()));
    connect(ui->confirm_params,SIGNAL(clicked()),this,SLOT(action_redraw()));
    connect(ui->toggle_wind,SIGNAL(stateChanged(int)),this,SLOT(action_toggle_wind()));
    connect(ui->wind_force_slider,SIGNAL(valueChanged(int)),this,SLOT(action_wind_force_changed()));
    connect(ui->sphere_scale,SIGNAL(valueChanged(int)),this,SLOT(action_scale_changed()));
    connect(ui->restart,SIGNAL(clicked()),this,SLOT(action_restart_simulation()));

}

myWindow::~myWindow()
{}

void myWindow::action_quit()
{
    close();
}

void myWindow::action_draw()
{
    glWidget->change_draw_state();
}

void myWindow::action_wireframe()
{
    bool const state_wireframe=ui->wireframe->isChecked();
    glWidget->wireframe(state_wireframe);
}

void myWindow::action_redraw(){

    float k_bend = ui->k_bending_edit->toPlainText().toFloat();
    float k_shearing = ui->k_shearing_edit->toPlainText().toFloat();
    float k_structural = ui->k_structural_edit->toPlainText().toFloat();
    glWidget->get_scene().change_k_params(k_structural,k_shearing,k_bend);

    //Activate depth buffer
    glEnable(GL_DEPTH_TEST);
}

void myWindow::action_confirm_bending(float const& k){
    QString k_bend = ui->k_bending_edit->toPlainText();
    glWidget->get_scene().get_mesh_cloth().set_k_bend(k_bend.toFloat());
}

void myWindow::action_confirm_shearing(float const& k){
    QString k_shearing = ui->k_shearing_edit->toPlainText();
    glWidget->get_scene().get_mesh_cloth().set_k_shear(k_shearing.toFloat());
}

void myWindow::action_confirm_structural(float const& k){
    QString k_struct = ui->k_structural_edit->toPlainText();
    glWidget->get_scene().get_mesh_cloth().set_k_shear(k_struct.toFloat());
}

void myWindow::action_toggle_wind(){
    glWidget->get_scene().toggle_wind();
    ui->toggle_wind->setChecked(glWidget->get_scene().get_wind());
}

void myWindow::action_wind_force_changed(){
    glWidget->get_scene().set_wind_force(
                ui->wind_force_slider->value());
    ui->wind_force_desc->setText(
                QString("wind force : ").append(QString::number(ui->wind_force_slider->value())));
}

void myWindow::action_scale_changed()
{
    glWidget->get_scene().set_sphere_radius((float)ui->sphere_scale->value()/10.0f);
    ui->sphere_scale_desc->setText(
                QString("Sphere scale : ").append(QString::number(ui->sphere_scale->value())));
}


void myWindow::action_update_fps(){
    //ui->fps->setText(QString("Fps : ").append(QString::number(glWidget->get_scene().fps)));
}
