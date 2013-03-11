#include "renderer/lights/light_dir_handles.h"
#include "renderer/lights/light_dir.h"
#include "renderer/shader/shader_program.h"
#include "renderer/renderer.h"
#include "jtil/exceptions/wruntime_error.h"

using std::wstring;
using std::wruntime_error;

namespace renderer {

  void LightDirHandles::getHandles(ShaderProgram* sp) {
    if (!sp->linked()) {
      throw wruntime_error(wstring(L"LightDirHandles::getHandles() - ") + 
        wstring(L"Error, sp is not yet linked!"));
    }
    h_color = sp->getUniformLocation("light.color");
    h_ambient_intensity = sp->getUniformLocation("light.ambient_intensity");
    h_diffuse_intensity = sp->getUniformLocation("light.diffuse_intensity");
    h_direction_view = sp->getUniformLocation("light.direction_view");
  }

  void LightDirHandles::setHandles(LightDir* light, Renderer* render) {
    LightDirData* light_data =  light->light_data();
    render->bindFloat3(&light_data->color, h_color);
    render->bindFloat1(light_data->ambient_intensity, h_ambient_intensity);
    render->bindFloat1(light_data->diffuse_intensity, h_diffuse_intensity);
    light_data->direction_view.normalize();  // just in case!
    render->bindFloat3(&light_data->direction_view, h_direction_view);
  }

}