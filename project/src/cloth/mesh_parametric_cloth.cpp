/*
**    TP CPE Lyon
**    Copyright (C) 2015 Damien Rohmer
**
**    This program is free software: you can redistribute it and/or modify
**    it under the terms of the GNU General Public License as published by
**    the Free Software Foundation, either version 3 of the License, or
**    (at your option) any later version.
**
**   This program is distributed in the hope that it will be useful,
**    but WITHOUT ANY WARRANTY; without even the implied warranty of
**    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**    GNU General Public License for more details.
**
**    You should have received a copy of the GNU General Public License
**    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "mesh_parametric_cloth.hpp"

#include "../lib/common/error_handling.hpp"
#include <cmath>
#include <cstdlib>
#include "../local/scene/scene.hpp"

namespace cpe
{


void mesh_parametric_cloth::update_force(float& h, bool& wind, int wind_force, float radius, vec3 center)
{

    int const Nu = size_u();
    int const Nv = size_v();
    int const N_total = Nu*Nv;

    ASSERT_CPE(static_cast<int>(force_data.size()) == Nu*Nv , "Error of size");


    //Gravity
    static vec3 const g (0.0f,0.0f,-9.81f);
    vec3 const g_normalized = g/N_total;

    for(int ku=0 ; ku<Nu ; ++ku)
        {
            for(int kv=0  ; kv<Nv ; ++kv)
            {
                force(ku,kv) = g_normalized;
                compute_spring_forces(Nu, Nv, ku, kv);

                if(vertex(ku,kv).z() < h + 0.0005f){
                    vertex(ku,kv).z() += 0.0005f;

                    vec3 normal = vertex(ku,kv) - vec3(0.0f,0.0f,-1.101f);
                    cpe::normalized(normal);
                    vec3 speed_normal = dot(speed(ku,kv),normal)*normal * (1- exp(-(abs(vertex(ku,kv).z() - h))*1000.0f));
                    vec3 speed_tangent = (speed(ku,kv) - speed_normal) * (1- exp(-(abs(vertex(ku,kv).z() - h))*1000.0f));
                    speed(ku,kv) = speed_tangent - speed_normal;
                }

                //Sphere collision
                if(norm(vertex(ku,kv) - center) < radius)
                {
                    vertex(ku,kv) = vertex(ku,kv) + vec3(0.0005f,0.0005f,0.0005f);

                    vec3 normal = vertex(ku,kv) - vec3(0.5f,0.05f,-1.1f);
                    cpe::normalized(normal);
                    vec3 speed_normal = dot(speed(ku,kv),normal)*normal;
                    vec3 speed_tangent = speed(ku,kv) - speed_normal;
                    speed(ku,kv) = speed_tangent - speed_normal;
                }

                //Wind force
                if(wind){
                    vec3 wind_direction = vec3(1.0f,0.0f,0.0f);
                    int random = rand() % 5000;
                    float K_wind = (float)random/100.0f * (float)wind_force/10000.0f;
                    vec3 wind_force = K_wind*dot(normal(ku,kv),wind_direction)*normal(ku,kv);
                    force(ku,kv) = force(ku,kv) + wind_force;
                }

            }
        }

        force(0,0)=vec3(0.0f, 0.0f, 0.0f);
        force(0,Nv-1)=vec3(0.0f, 0.0f, 0.0f);
}

void mesh_parametric_cloth::integration_step(float const& dt)
{
    ASSERT_CPE(speed_data.size() == force_data.size(),"Incorrect size");
    ASSERT_CPE(static_cast<int>(speed_data.size()) == size_vertex(),"Incorrect size");


    int const Nu = size_u();
    int const Nv = size_v();

    //security check (throw exception if divergence is detected)
    static float const LIMIT=30.0f;
    //static float const K_LIMIT = 35.0f;
    for(int ku=0 ; ku<Nu ; ++ku)
    {
        for(int kv=0 ; kv<Nv ; ++kv)
        {
            vec3& p = vertex(ku,kv);
            speed(ku,kv) = (1-0.4f*dt)*speed(ku,kv) + dt*force(ku,kv);
            p = p + speed(ku,kv)*dt;

            /*if(norm(p) > LIMIT - 25.0f && k_bending > 0.1f && k_shearing > 0.1f && k_structural > 0.1f)
            {
                k_bending -= 1.0f;
                k_shearing -= 1.0f;
                k_structural -= 1.0f;
            }
            else if(norm(p) < LIMIT - 25.0f && k_bending < K_LIMIT && k_shearing < K_LIMIT && k_structural < K_LIMIT){
                k_bending += 0.2f;
                k_shearing += 0.2f;
                k_structural += 0.2f;
            }*/
            if( norm(p) > LIMIT )
            {
                std::cout << "norme de p : " << norm(p) << std::endl;
                throw exception_divergence("Divergence of the system",EXCEPTION_PARAMETERS_CPE);
            }
        }
    }

}

void mesh_parametric_cloth::compute_spring_forces(const int &Nu, const int &Nv, int &ku, int &kv){
    //structural string
    float L_structural=1.0f/(Nu-1);

    if (ku<Nu-1)
    {
        vec3 u1 = vertex(ku,kv)-vertex(ku+1,kv);
        force(ku,kv) += k_structural*(L_structural-norm(u1))*u1/norm(u1);
    }
    if (kv<Nv-1)
    {
        vec3 u2 = vertex(ku,kv)-vertex(ku,kv+1);
        force(ku,kv) += k_structural*(L_structural-norm(u2))*u2/norm(u2);
    }
    if (ku>0)
    {
        vec3 u3 = vertex(ku,kv)-vertex(ku-1,kv);
        force(ku,kv) += k_structural*(L_structural-norm(u3))*u3/norm(u3);
    }
    if (kv>0)
    {
        vec3 u4 = vertex(ku,kv)-vertex(ku,kv-1);
        force(ku,kv) += k_structural*(L_structural-norm(u4))*u4/norm(u4);
    }

    //shearing springs
    float L_shearing=sqrt(2)*L_structural;

    if (ku<Nu-1 && kv<Nv-1)
    {
        vec3 u5 = vertex(ku,kv)-vertex(ku+1,kv+1);
        force(ku,kv) += k_shearing*(L_shearing-norm(u5))*u5/norm(u5);
    }
    if (ku>0 && kv<Nv-1)
    {
        vec3 u6 = vertex(ku,kv)-vertex(ku-1,kv+1);
        force(ku,kv) += k_shearing*(L_shearing-norm(u6))*u6/norm(u6);
    }
    if (kv>0 && ku<Nu-1)
    {
        vec3 u7 = vertex(ku,kv)-vertex(ku+1,kv-1);
        force(ku,kv) += k_shearing*(L_shearing-norm(u7))*u7/norm(u7);
    }
    if (ku>0 && kv>0)
    {
        vec3 u8 = vertex(ku,kv)-vertex(ku-1,kv-1);
        force(ku,kv) += k_shearing*(L_shearing-norm(u8))*u8/norm(u8);
    }

    //bending springs
    float L_bending=2*L_structural;

    if (ku<Nu-2)
    {
        vec3 u9 = vertex(ku,kv)-vertex(ku+2,kv);
        force(ku,kv) += k_bending*(L_bending-norm(u9))*u9/norm(u9);
    }
    if (kv<Nv-2)
    {
        vec3 u10 = vertex(ku,kv)-vertex(ku,kv+2);
        force(ku,kv) += k_bending*(L_bending-norm(u10))*u10/norm(u10);
    }
    if (ku>1)
    {
        vec3 u11 = vertex(ku,kv)-vertex(ku-2,kv);
        force(ku,kv) += k_bending*(L_bending-norm(u11))*u11/norm(u11);
    }
    if (kv>1)
    {
        vec3 u12 = vertex(ku,kv)-vertex(ku,kv-2);
        force(ku,kv) += k_bending*(L_bending-norm(u12))*u12/norm(u12);
    }
}

void mesh_parametric_cloth::compute_single_spring_force(float k, float L, int ku, int kv, int adj_ku, int adj_kv){
    vec3 u = vertex(ku,kv) - vertex(adj_ku,adj_kv);
    force(ku,kv) = k*(L - norm(u))*u/norm(u);
}

void mesh_parametric_cloth::set_k_struct(float const& k){
    k_structural = k;
}

void mesh_parametric_cloth::set_k_shear(float const& k){
    k_shearing = k;
}

void mesh_parametric_cloth::set_k_bend(float const& k){
    k_bending = k;
}

std::string mesh_parametric_cloth::str_k_struct(){
    return std::to_string(k_structural);
}

std::string mesh_parametric_cloth::str_k_shear(){
    return std::to_string(k_shearing);
}

std::string mesh_parametric_cloth::str_k_bend(){
    return std::to_string(k_bending);
}

void mesh_parametric_cloth::set_plane_xy_unit(int const size_u_param,int const size_v_param)
{
    mesh_parametric::set_plane_xy_unit(size_u_param,size_v_param);

    int const N = size_u()*size_v();
    speed_data.resize(N);
    force_data.resize(N);
}

vec3 const& mesh_parametric_cloth::speed(int const ku,int const kv) const
{
    ASSERT_CPE(ku >= 0 , "Value ku ("+std::to_string(ku)+") should be >=0 ");
    ASSERT_CPE(ku < size_u() , "Value ku ("+std::to_string(ku)+") should be < size_u ("+std::to_string(size_u())+")");
    ASSERT_CPE(kv >= 0 , "Value kv ("+std::to_string(kv)+") should be >=0 ");
    ASSERT_CPE(kv < size_v() , "Value kv ("+std::to_string(kv)+") should be < size_v ("+std::to_string(size_v())+")");

    int const offset = ku + size_u()*kv;

    ASSERT_CPE(offset < static_cast<int>(speed_data.size()),"Internal error");

    return speed_data[offset];
}

vec3& mesh_parametric_cloth::speed(int const ku,int const kv)
{
    ASSERT_CPE(ku >= 0 , "Value ku ("+std::to_string(ku)+") should be >=0 ");
    ASSERT_CPE(ku < size_u() , "Value ku ("+std::to_string(ku)+") should be < size_u ("+std::to_string(size_u())+")");
    ASSERT_CPE(kv >= 0 , "Value kv ("+std::to_string(kv)+") should be >=0 ");
    ASSERT_CPE(kv < size_v() , "Value kv ("+std::to_string(kv)+") should be < size_v ("+std::to_string(size_v())+")");

    int const offset = ku + size_u()*kv;

    ASSERT_CPE(offset < static_cast<int>(speed_data.size()),"Internal error");

    return speed_data[offset];
}

vec3 const& mesh_parametric_cloth::force(int const ku,int const kv) const
{
    ASSERT_CPE(ku >= 0 , "Value ku ("+std::to_string(ku)+") should be >=0 ");
    ASSERT_CPE(ku < size_u() , "Value ku ("+std::to_string(ku)+") should be < size_u ("+std::to_string(size_u())+")");
    ASSERT_CPE(kv >= 0 , "Value kv ("+std::to_string(kv)+") should be >=0 ");
    ASSERT_CPE(kv < size_v() , "Value kv ("+std::to_string(kv)+") should be < size_v ("+std::to_string(size_v())+")");

    int const offset = ku + size_u()*kv;

    ASSERT_CPE(offset < static_cast<int>(force_data.size()),"Internal error");

    return force_data[offset];
}

vec3& mesh_parametric_cloth::force(int const ku,int const kv)
{
    ASSERT_CPE(ku >= 0 , "Value ku ("+std::to_string(ku)+") should be >=0 ");
    ASSERT_CPE(ku < size_u() , "Value ku ("+std::to_string(ku)+") should be < size_u ("+std::to_string(size_u())+")");
    ASSERT_CPE(kv >= 0 , "Value kv ("+std::to_string(kv)+") should be >=0 ");
    ASSERT_CPE(kv < size_v() , "Value kv ("+std::to_string(kv)+") should be < size_v ("+std::to_string(size_v())+")");

    int const offset = ku + size_u()*kv;

    ASSERT_CPE(offset < static_cast<int>(force_data.size()),"Internal error");

    return force_data[offset];
}


}
