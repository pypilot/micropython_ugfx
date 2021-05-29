/* Copyright (C) 2021 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

// micropython ugfx driver for esp32 and ili9341

#include "sdkconfig.h"

#include <stdint.h>

#include "py/obj.h"
#include "py/runtime.h"

#include "upy_wrap.h"

typedef struct _ugfx_surface_obj_t {
    mp_obj_base_t base;
    void *ptr;
    int contrast, hue;
    int backlight;
} ugfx_surface_obj_t;
const mp_obj_type_t ugfx_surface_type;

STATIC mp_obj_t ugfx_surface_load(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum {ARG_filename, ARG_bypp};
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_filename, MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_bypp, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } }
    };

    ugfx_surface_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_obj_t *filename = MP_OBJ_TO_PTR(args[ARG_filename].u_obj);

    char *path = (char *)mp_obj_str_get_str(filename);
    mp_int_t bypp = args[ARG_bypp].u_int;

    ugfx_surface_c_load(self->ptr, path, bypp);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ugfx_surface_load_obj, 0, ugfx_surface_load);

STATIC mp_obj_t ugfx_surface_blit(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum {ARG_src, ARG_xoff, ARG_yoff, ARG_flip};
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_src, MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_xoff, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_yoff, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_flip, MP_ARG_KW_ONLY | MP_ARG_BOOL, { .u_bool = false } },
    };

    ugfx_surface_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    ugfx_surface_obj_t *src = MP_OBJ_TO_PTR(args[ARG_src].u_obj);
    mp_int_t xoff = args[ARG_xoff].u_int;
    mp_int_t yoff = args[ARG_yoff].u_int;
    uint8_t flip = args[ARG_flip].u_bool;

    ugfx_surface_c_blit(self->ptr, src->ptr, xoff, yoff, flip);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ugfx_surface_blit_obj, 0, ugfx_surface_blit);

STATIC mp_obj_t ugfx_surface_line(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum {ARG_x1, ARG_y1, ARG_x2, ARG_y2, ARG_c};
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_x1, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y1, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_x2, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y2, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_c, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
    };

    ugfx_surface_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_int_t x1 = args[ARG_x1].u_int;
    mp_int_t y1 = args[ARG_y1].u_int;
    mp_int_t x2 = args[ARG_x2].u_int;
    mp_int_t y2 = args[ARG_y2].u_int;
    mp_int_t c = args[ARG_c].u_int;

    ugfx_surface_c_line(self->ptr, x1, y1, x2, y2, c);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ugfx_surface_line_obj, 0, ugfx_surface_line);

STATIC mp_obj_t ugfx_surface_box(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum {ARG_x1, ARG_y1, ARG_x2, ARG_y2, ARG_c};
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_x1, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y1, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_x2, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y2, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_c, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
    };

    ugfx_surface_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_int_t x1 = args[ARG_x1].u_int;
    mp_int_t y1 = args[ARG_y1].u_int;
    mp_int_t x2 = args[ARG_x2].u_int;
    mp_int_t y2 = args[ARG_y2].u_int;
    mp_int_t c = args[ARG_c].u_int;

    ugfx_surface_c_box(self->ptr, x1, y1, x2, y2, c);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ugfx_surface_box_obj, 0, ugfx_surface_box);

STATIC mp_obj_t ugfx_surface_invert(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum {ARG_x1, ARG_y1, ARG_x2, ARG_y2};
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_x1, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y1, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_x2, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y2, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
    };

    ugfx_surface_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_int_t x1 = args[ARG_x1].u_int;
    mp_int_t y1 = args[ARG_y1].u_int;
    mp_int_t x2 = args[ARG_x2].u_int;
    mp_int_t y2 = args[ARG_y2].u_int;

    ugfx_surface_c_invert(self->ptr, x1, y1, x2, y2);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ugfx_surface_invert_obj, 0, ugfx_surface_invert);

STATIC mp_obj_t ugfx_surface_fill(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum {ARG_c};
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_c, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
    };

    ugfx_surface_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_int_t c = args[ARG_c].u_int;

    ugfx_surface_c_fill(self->ptr, c);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ugfx_surface_fill_obj, 0, ugfx_surface_fill);


void ili_refresh(int contrast, int hue, int backlight, int width, int height, char *data);

STATIC mp_obj_t ugfx_display_surface_refresh(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    ugfx_surface_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    int width, height, bypp;
    char *data;
    ugfx_surface_c_info(self->ptr, &width, &height, &bypp, &data);

    ili_refresh(self->contrast, self->hue, self->backlight, width, height, data);

    return mp_const_none;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ugfx_display_surface_refresh_obj, 0, ugfx_display_surface_refresh);

STATIC mp_obj_t ugfx_surface_free(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    ugfx_surface_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    ugfx_surface_c_free(self->ptr);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ugfx_surface_free_obj, 0, ugfx_surface_free);


STATIC const mp_rom_map_elem_t ugfx_surface_locals_dict_table[] = {
    // instance methods
    { MP_ROM_QSTR(MP_QSTR_load),                MP_ROM_PTR(&ugfx_surface_load_obj) },
    { MP_ROM_QSTR(MP_QSTR_blit),                MP_ROM_PTR(&ugfx_surface_blit_obj) },
    { MP_ROM_QSTR(MP_QSTR_line),                MP_ROM_PTR(&ugfx_surface_line_obj) },
    { MP_ROM_QSTR(MP_QSTR_box),                 MP_ROM_PTR(&ugfx_surface_box_obj) },
    { MP_ROM_QSTR(MP_QSTR_invert),              MP_ROM_PTR(&ugfx_surface_invert_obj) },
    { MP_ROM_QSTR(MP_QSTR_fill),                MP_ROM_PTR(&ugfx_surface_fill_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh),             MP_ROM_PTR(&ugfx_display_surface_refresh_obj) }
};
STATIC MP_DEFINE_CONST_DICT(ugfx_surface_locals_dict, ugfx_surface_locals_dict_table);


//-----------------------------------------------------------------------------------------------
STATIC void ugfx_surface_printinfo(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    ugfx_surface_obj_t *self = self_in;
    int width, height, bypp;
    char *data;
    ugfx_surface_c_info(self->ptr, &width, &height, &bypp, &data);
    mp_printf(print, "surface %dx%dx%d %p", width, height, bypp, data);
}

// constructor(id, ...)
//-----------------------------------------------------------------------------------------------------------------
STATIC mp_obj_t ugfx_surface_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {    
    // this checks the number of arguments (min 1, max 1);
    // on error -> raise python exception
    ugfx_surface_obj_t *self = m_new_obj(ugfx_surface_obj_t);
    self->base.type = &ugfx_surface_type;

    if(n_args == 2) {
        mp_arg_check_num(n_args, n_kw, 1, 2, true);
        char *path = (char *)mp_obj_str_get_str(args[0]);
        self->ptr = ugfx_surface_from_file_c(path, mp_obj_get_int(args[1]));
        return MP_OBJ_FROM_PTR(self);
    }

    mp_arg_check_num(n_args, n_kw, 1, 4, true);

    int w = mp_obj_get_int(args[0]);
    int h = mp_obj_get_int(args[1]);
    int bypp = mp_obj_get_int(args[2]);

    self->contrast = 60;
    self->hue = 170;
    self->backlight = 1;
    self->ptr = ugfx_surface_from_data_c(w, h, bypp, 0);

    return MP_OBJ_FROM_PTR(self);
}

STATIC void ugfx_surface_attr(mp_obj_t self_in, qstr attribute, mp_obj_t *dest) {
    ugfx_surface_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int width, height, bypp;
    char *data;
    ugfx_surface_c_info(self->ptr, &width, &height, &bypp, &data);
    if(attribute == MP_QSTR_width)
        dest[0] = mp_obj_new_int(width);
    else
    if(attribute == MP_QSTR_height)
        dest[0] = mp_obj_new_int(height);
    else
    if(attribute == MP_QSTR_bypp)
        dest[0] = mp_obj_new_int(bypp);
    else
    if(attribute == MP_QSTR_contrast)
        if(dest[0] == MP_OBJ_SENTINEL) {
            self->contrast = mp_obj_get_int(dest[1]);
            dest[0] = MP_OBJ_NULL;
        } else
            dest[0] = mp_obj_new_int(self->contrast);
    else
    if(attribute == MP_QSTR_hue)
        if(dest[0] == MP_OBJ_SENTINEL) {
            self->hue = mp_obj_get_int(dest[1]);
            dest[0] = MP_OBJ_NULL;
        } else
            dest[0] = mp_obj_new_int(self->hue);
    else
    if(attribute == MP_QSTR_backlight)
        if(dest[0] == MP_OBJ_SENTINEL) {
            self->backlight = mp_obj_get_int(dest[1]);
            dest[0] = MP_OBJ_NULL;
        } else
            dest[0] = mp_obj_new_int(self->backlight);
    else {
        // fallback to lookup
        const mp_obj_type_t *type = mp_obj_get_type(self_in);
        mp_map_t *locals_map = &type->locals_dict->map;
        mp_map_elem_t *elem = mp_map_lookup(locals_map, MP_OBJ_NEW_QSTR(attribute), MP_MAP_LOOKUP);
        if (elem != NULL)
            mp_convert_member_lookup(self_in, type, elem->value, dest);
    }
}

const mp_obj_type_t ugfx_surface_type = {
    { &mp_type_type },
    .name = MP_QSTR_surface,
    .print = ugfx_surface_printinfo,
    .make_new = ugfx_surface_make_new,
    .attr = ugfx_surface_attr,
    .locals_dict = (mp_obj_t)&ugfx_surface_locals_dict,
};

typedef struct _ugfx_obj_t {
    mp_obj_base_t base;
} ugfx_obj_t;

//===============================================================
STATIC const mp_rom_map_elem_t ugfx_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_ugfx) },
    { MP_ROM_QSTR(MP_QSTR_surface),                MP_ROM_PTR(&ugfx_surface_type) },
};

//===============================================================================
STATIC MP_DEFINE_CONST_DICT(ugfx_module_globals, ugfx_module_globals_table);

const mp_obj_module_t mp_module_ugfx = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&ugfx_module_globals,
};

// Register the module to make it available in Python
MP_REGISTER_MODULE(MP_QSTR_ugfx, mp_module_ugfx, MODULE_UGFX_ENABLED);
