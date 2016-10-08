/* C++ Interface to GNUPLOT
 *
 * FILE: gnuplot_ci.h
 *
 * Date: 09 August 2007 Thursday
 * Date: 12 October 2007 Friday
 * ---------------------------------- */
#ifndef _GNUPLOT_CI_H
#define _GNUPLOT_CI_H

#include<cstdio>
#include<cstdlib>
#include<cstring>
#include<unistd.h>
#include<csignal>
#include<cmath>
#include <cstring>

namespace gnuplot_ci
{
  class GP_handle
  {
    private:
      char *gp_binary;
      char *set_term;
      char *xt;
      char *yt;
      char *zt;

      FILE *fp;

    public:
      GP_handle(const char *gp_bin, const char *xl, const char *yl, const char *zl=NULL);
      GP_handle(const char *gp_bin);
      ~GP_handle();
      void gnuplot_cmd(const char *command);
      void gnuplot_image(const char *image_address,const char *filetype);
      void draw_circle(float x, float y, float r);
      void draw_sphere(float x, float y, float z, float r, int iso, int ltc=0);
      void draw_cuboid(float x1, float y1, float z1, float x2, float y2, float z2);
      void draw_cuboid2(float x0, float y0, float z0, float dx, float dy, float dz, int replot_flag=0);
  };
}





#endif
