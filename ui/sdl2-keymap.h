
/* map SDL2 scancodes to QKeyCode */

static const int sdl2_scancode_to_qcode[SDL_NUM_SCANCODES] = {
    [SDL_SCANCODE_A]                 = Q_KEY_CODE_A,
    [SDL_SCANCODE_B]                 = Q_KEY_CODE_B,
    [SDL_SCANCODE_C]                 = Q_KEY_CODE_C,
    [SDL_SCANCODE_D]                 = Q_KEY_CODE_D,
    [SDL_SCANCODE_E]                 = Q_KEY_CODE_E,
    [SDL_SCANCODE_F]                 = Q_KEY_CODE_F,
    [SDL_SCANCODE_G]                 = Q_KEY_CODE_G,
    [SDL_SCANCODE_H]                 = Q_KEY_CODE_H,
    [SDL_SCANCODE_I]                 = Q_KEY_CODE_I,
    [SDL_SCANCODE_J]                 = Q_KEY_CODE_J,
    [SDL_SCANCODE_K]                 = Q_KEY_CODE_K,
    [SDL_SCANCODE_L]                 = Q_KEY_CODE_L,
    [SDL_SCANCODE_M]                 = Q_KEY_CODE_M,
    [SDL_SCANCODE_N]                 = Q_KEY_CODE_N,
    [SDL_SCANCODE_O]                 = Q_KEY_CODE_O,
    [SDL_SCANCODE_P]                 = Q_KEY_CODE_P,
    [SDL_SCANCODE_Q]                 = Q_KEY_CODE_Q,
    [SDL_SCANCODE_R]                 = Q_KEY_CODE_R,
    [SDL_SCANCODE_S]                 = Q_KEY_CODE_S,
    [SDL_SCANCODE_T]                 = Q_KEY_CODE_T,
    [SDL_SCANCODE_U]                 = Q_KEY_CODE_U,
    [SDL_SCANCODE_V]                 = Q_KEY_CODE_V,
    [SDL_SCANCODE_W]                 = Q_KEY_CODE_W,
    [SDL_SCANCODE_X]                 = Q_KEY_CODE_X,
    [SDL_SCANCODE_Y]                 = Q_KEY_CODE_Y,
    [SDL_SCANCODE_Z]                 = Q_KEY_CODE_Z,

    [SDL_SCANCODE_1]                 = Q_KEY_CODE_1,
    [SDL_SCANCODE_2]                 = Q_KEY_CODE_2,
    [SDL_SCANCODE_3]                 = Q_KEY_CODE_3,
    [SDL_SCANCODE_4]                 = Q_KEY_CODE_4,
    [SDL_SCANCODE_5]                 = Q_KEY_CODE_5,
    [SDL_SCANCODE_6]                 = Q_KEY_CODE_6,
    [SDL_SCANCODE_7]                 = Q_KEY_CODE_7,
    [SDL_SCANCODE_8]                 = Q_KEY_CODE_8,
    [SDL_SCANCODE_9]                 = Q_KEY_CODE_9,
    [SDL_SCANCODE_0]                 = Q_KEY_CODE_0,

    [SDL_SCANCODE_RETURN]            = Q_KEY_CODE_RET,
    [SDL_SCANCODE_ESCAPE]            = Q_KEY_CODE_ESC,
    [SDL_SCANCODE_BACKSPACE]         = Q_KEY_CODE_BACKSPACE,
    [SDL_SCANCODE_TAB]               = Q_KEY_CODE_TAB,
    [SDL_SCANCODE_SPACE]             = Q_KEY_CODE_SPC,
    [SDL_SCANCODE_MINUS]             = Q_KEY_CODE_MINUS,
    [SDL_SCANCODE_EQUALS]            = Q_KEY_CODE_EQUAL,
    [SDL_SCANCODE_LEFTBRACKET]       = Q_KEY_CODE_BRACKET_LEFT,
    [SDL_SCANCODE_RIGHTBRACKET]      = Q_KEY_CODE_BRACKET_RIGHT,
    [SDL_SCANCODE_BACKSLASH]         = Q_KEY_CODE_BACKSLASH,
#if 0
    [SDL_SCANCODE_NONUSHASH]         = Q_KEY_CODE_NONUSHASH,
#endif
    [SDL_SCANCODE_SEMICOLON]         = Q_KEY_CODE_SEMICOLON,
    [SDL_SCANCODE_APOSTROPHE]        = Q_KEY_CODE_APOSTROPHE,
    [SDL_SCANCODE_GRAVE]             = Q_KEY_CODE_GRAVE_ACCENT,
    [SDL_SCANCODE_COMMA]             = Q_KEY_CODE_COMMA,
    [SDL_SCANCODE_PERIOD]            = Q_KEY_CODE_DOT,
    [SDL_SCANCODE_SLASH]             = Q_KEY_CODE_SLASH,
    [SDL_SCANCODE_CAPSLOCK]          = Q_KEY_CODE_CAPS_LOCK,

    [SDL_SCANCODE_F1]                = Q_KEY_CODE_F1,
    [SDL_SCANCODE_F2]                = Q_KEY_CODE_F2,
    [SDL_SCANCODE_F3]                = Q_KEY_CODE_F3,
    [SDL_SCANCODE_F4]                = Q_KEY_CODE_F4,
    [SDL_SCANCODE_F5]                = Q_KEY_CODE_F5,
    [SDL_SCANCODE_F6]                = Q_KEY_CODE_F6,
    [SDL_SCANCODE_F7]                = Q_KEY_CODE_F7,
    [SDL_SCANCODE_F8]                = Q_KEY_CODE_F8,
    [SDL_SCANCODE_F9]                = Q_KEY_CODE_F9,
    [SDL_SCANCODE_F10]               = Q_KEY_CODE_F10,
    [SDL_SCANCODE_F11]               = Q_KEY_CODE_F11,
    [SDL_SCANCODE_F12]               = Q_KEY_CODE_F12,

    [SDL_SCANCODE_PRINTSCREEN]       = Q_KEY_CODE_PRINT,
    [SDL_SCANCODE_SCROLLLOCK]        = Q_KEY_CODE_SCROLL_LOCK,
    [SDL_SCANCODE_PAUSE]             = Q_KEY_CODE_PAUSE,
    [SDL_SCANCODE_INSERT]            = Q_KEY_CODE_INSERT,
    [SDL_SCANCODE_HOME]              = Q_KEY_CODE_HOME,
    [SDL_SCANCODE_PAGEUP]            = Q_KEY_CODE_PGUP,
    [SDL_SCANCODE_DELETE]            = Q_KEY_CODE_DELETE,
    [SDL_SCANCODE_END]               = Q_KEY_CODE_END,
    [SDL_SCANCODE_PAGEDOWN]          = Q_KEY_CODE_PGDN,
    [SDL_SCANCODE_RIGHT]             = Q_KEY_CODE_RIGHT,
    [SDL_SCANCODE_LEFT]              = Q_KEY_CODE_LEFT,
    [SDL_SCANCODE_DOWN]              = Q_KEY_CODE_DOWN,
    [SDL_SCANCODE_UP]                = Q_KEY_CODE_UP,
    [SDL_SCANCODE_NUMLOCKCLEAR]      = Q_KEY_CODE_NUM_LOCK,

    [SDL_SCANCODE_KP_DIVIDE]         = Q_KEY_CODE_KP_DIVIDE,
    [SDL_SCANCODE_KP_MULTIPLY]       = Q_KEY_CODE_KP_MULTIPLY,
    [SDL_SCANCODE_KP_MINUS]          = Q_KEY_CODE_KP_SUBTRACT,
    [SDL_SCANCODE_KP_PLUS]           = Q_KEY_CODE_KP_ADD,
    [SDL_SCANCODE_KP_ENTER]          = Q_KEY_CODE_KP_ENTER,
    [SDL_SCANCODE_KP_1]              = Q_KEY_CODE_KP_1,
    [SDL_SCANCODE_KP_2]              = Q_KEY_CODE_KP_2,
    [SDL_SCANCODE_KP_3]              = Q_KEY_CODE_KP_3,
    [SDL_SCANCODE_KP_4]              = Q_KEY_CODE_KP_4,
    [SDL_SCANCODE_KP_5]              = Q_KEY_CODE_KP_5,
    [SDL_SCANCODE_KP_6]              = Q_KEY_CODE_KP_6,
    [SDL_SCANCODE_KP_7]              = Q_KEY_CODE_KP_7,
    [SDL_SCANCODE_KP_8]              = Q_KEY_CODE_KP_8,
    [SDL_SCANCODE_KP_9]              = Q_KEY_CODE_KP_9,
    [SDL_SCANCODE_KP_0]              = Q_KEY_CODE_KP_0,
    [SDL_SCANCODE_KP_PERIOD]         = Q_KEY_CODE_KP_DECIMAL,
#if 0
    [SDL_SCANCODE_NONUSBACKSLASH]    = Q_KEY_CODE_NONUSBACKSLASH,
    [SDL_SCANCODE_APPLICATION]       = Q_KEY_CODE_APPLICATION,
    [SDL_SCANCODE_POWER]             = Q_KEY_CODE_POWER,
    [SDL_SCANCODE_KP_EQUALS]         = Q_KEY_CODE_KP_EQUALS,

    [SDL_SCANCODE_F13]               = Q_KEY_CODE_F13,
    [SDL_SCANCODE_F14]               = Q_KEY_CODE_F14,
    [SDL_SCANCODE_F15]               = Q_KEY_CODE_F15,
    [SDL_SCANCODE_F16]               = Q_KEY_CODE_F16,
    [SDL_SCANCODE_F17]               = Q_KEY_CODE_F17,
    [SDL_SCANCODE_F18]               = Q_KEY_CODE_F18,
    [SDL_SCANCODE_F19]               = Q_KEY_CODE_F19,
    [SDL_SCANCODE_F20]               = Q_KEY_CODE_F20,
    [SDL_SCANCODE_F21]               = Q_KEY_CODE_F21,
    [SDL_SCANCODE_F22]               = Q_KEY_CODE_F22,
    [SDL_SCANCODE_F23]               = Q_KEY_CODE_F23,
    [SDL_SCANCODE_F24]               = Q_KEY_CODE_F24,

    [SDL_SCANCODE_EXECUTE]           = Q_KEY_CODE_EXECUTE,
#endif
    [SDL_SCANCODE_HELP]              = Q_KEY_CODE_HELP,
    [SDL_SCANCODE_MENU]              = Q_KEY_CODE_MENU,
#if 0
    [SDL_SCANCODE_SELECT]            = Q_KEY_CODE_SELECT,
#endif
    [SDL_SCANCODE_STOP]              = Q_KEY_CODE_STOP,
    [SDL_SCANCODE_AGAIN]             = Q_KEY_CODE_AGAIN,
    [SDL_SCANCODE_UNDO]              = Q_KEY_CODE_UNDO,
    [SDL_SCANCODE_CUT]               = Q_KEY_CODE_CUT,
    [SDL_SCANCODE_COPY]              = Q_KEY_CODE_COPY,
    [SDL_SCANCODE_PASTE]             = Q_KEY_CODE_PASTE,
    [SDL_SCANCODE_FIND]              = Q_KEY_CODE_FIND,
#if 0
    [SDL_SCANCODE_MUTE]              = Q_KEY_CODE_MUTE,
    [SDL_SCANCODE_VOLUMEUP]          = Q_KEY_CODE_VOLUMEUP,
    [SDL_SCANCODE_VOLUMEDOWN]        = Q_KEY_CODE_VOLUMEDOWN,

    [SDL_SCANCODE_KP_COMMA]          = Q_KEY_CODE_KP_COMMA,
    [SDL_SCANCODE_KP_EQUALSAS400]    = Q_KEY_CODE_KP_EQUALSAS400,

    [SDL_SCANCODE_INTERNATIONAL1]    = Q_KEY_CODE_INTERNATIONAL1,
    [SDL_SCANCODE_INTERNATIONAL2]    = Q_KEY_CODE_INTERNATIONAL2,
    [SDL_SCANCODE_INTERNATIONAL3]    = Q_KEY_CODE_INTERNATIONAL3,
    [SDL_SCANCODE_INTERNATIONAL4]    = Q_KEY_CODE_INTERNATIONAL4,
    [SDL_SCANCODE_INTERNATIONAL5]    = Q_KEY_CODE_INTERNATIONAL5,
    [SDL_SCANCODE_INTERNATIONAL6]    = Q_KEY_CODE_INTERNATIONAL6,
    [SDL_SCANCODE_INTERNATIONAL7]    = Q_KEY_CODE_INTERNATIONAL7,
    [SDL_SCANCODE_INTERNATIONAL8]    = Q_KEY_CODE_INTERNATIONAL8,
    [SDL_SCANCODE_INTERNATIONAL9]    = Q_KEY_CODE_INTERNATIONAL9,
    [SDL_SCANCODE_LANG1]             = Q_KEY_CODE_LANG1,
    [SDL_SCANCODE_LANG2]             = Q_KEY_CODE_LANG2,
    [SDL_SCANCODE_LANG3]             = Q_KEY_CODE_LANG3,
    [SDL_SCANCODE_LANG4]             = Q_KEY_CODE_LANG4,
    [SDL_SCANCODE_LANG5]             = Q_KEY_CODE_LANG5,
    [SDL_SCANCODE_LANG6]             = Q_KEY_CODE_LANG6,
    [SDL_SCANCODE_LANG7]             = Q_KEY_CODE_LANG7,
    [SDL_SCANCODE_LANG8]             = Q_KEY_CODE_LANG8,
    [SDL_SCANCODE_LANG9]             = Q_KEY_CODE_LANG9,
    [SDL_SCANCODE_ALTERASE]          = Q_KEY_CODE_ALTERASE,
#endif
    [SDL_SCANCODE_SYSREQ]            = Q_KEY_CODE_SYSRQ,
#if 0
    [SDL_SCANCODE_CANCEL]            = Q_KEY_CODE_CANCEL,
    [SDL_SCANCODE_CLEAR]             = Q_KEY_CODE_CLEAR,
    [SDL_SCANCODE_PRIOR]             = Q_KEY_CODE_PRIOR,
    [SDL_SCANCODE_RETURN2]           = Q_KEY_CODE_RETURN2,
    [SDL_SCANCODE_SEPARATOR]         = Q_KEY_CODE_SEPARATOR,
    [SDL_SCANCODE_OUT]               = Q_KEY_CODE_OUT,
    [SDL_SCANCODE_OPER]              = Q_KEY_CODE_OPER,
    [SDL_SCANCODE_CLEARAGAIN]        = Q_KEY_CODE_CLEARAGAIN,
    [SDL_SCANCODE_CRSEL]             = Q_KEY_CODE_CRSEL,
    [SDL_SCANCODE_EXSEL]             = Q_KEY_CODE_EXSEL,
    [SDL_SCANCODE_KP_00]             = Q_KEY_CODE_KP_00,
    [SDL_SCANCODE_KP_000]            = Q_KEY_CODE_KP_000,
    [SDL_SCANCODE_THOUSANDSSEPARATOR] = Q_KEY_CODE_THOUSANDSSEPARATOR,
    [SDL_SCANCODE_DECIMALSEPARATOR]  = Q_KEY_CODE_DECIMALSEPARATOR,
    [SDL_SCANCODE_CURRENCYUNIT]      = Q_KEY_CODE_CURRENCYUNIT,
    [SDL_SCANCODE_CURRENCYSUBUNIT]   = Q_KEY_CODE_CURRENCYSUBUNIT,
    [SDL_SCANCODE_KP_LEFTPAREN]      = Q_KEY_CODE_KP_LEFTPAREN,
    [SDL_SCANCODE_KP_RIGHTPAREN]     = Q_KEY_CODE_KP_RIGHTPAREN,
    [SDL_SCANCODE_KP_LEFTBRACE]      = Q_KEY_CODE_KP_LEFTBRACE,
    [SDL_SCANCODE_KP_RIGHTBRACE]     = Q_KEY_CODE_KP_RIGHTBRACE,
    [SDL_SCANCODE_KP_TAB]            = Q_KEY_CODE_KP_TAB,
    [SDL_SCANCODE_KP_BACKSPACE]      = Q_KEY_CODE_KP_BACKSPACE,
    [SDL_SCANCODE_KP_A]              = Q_KEY_CODE_KP_A,
    [SDL_SCANCODE_KP_B]              = Q_KEY_CODE_KP_B,
    [SDL_SCANCODE_KP_C]              = Q_KEY_CODE_KP_C,
    [SDL_SCANCODE_KP_D]              = Q_KEY_CODE_KP_D,
    [SDL_SCANCODE_KP_E]              = Q_KEY_CODE_KP_E,
    [SDL_SCANCODE_KP_F]              = Q_KEY_CODE_KP_F,
    [SDL_SCANCODE_KP_XOR]            = Q_KEY_CODE_KP_XOR,
    [SDL_SCANCODE_KP_POWER]          = Q_KEY_CODE_KP_POWER,
    [SDL_SCANCODE_KP_PERCENT]        = Q_KEY_CODE_KP_PERCENT,
    [SDL_SCANCODE_KP_LESS]           = Q_KEY_CODE_KP_LESS,
    [SDL_SCANCODE_KP_GREATER]        = Q_KEY_CODE_KP_GREATER,
    [SDL_SCANCODE_KP_AMPERSAND]      = Q_KEY_CODE_KP_AMPERSAND,
    [SDL_SCANCODE_KP_DBLAMPERSAND]   = Q_KEY_CODE_KP_DBLAMPERSAND,
    [SDL_SCANCODE_KP_VERTICALBAR]    = Q_KEY_CODE_KP_VERTICALBAR,
    [SDL_SCANCODE_KP_DBLVERTICALBAR] = Q_KEY_CODE_KP_DBLVERTICALBAR,
    [SDL_SCANCODE_KP_COLON]          = Q_KEY_CODE_KP_COLON,
    [SDL_SCANCODE_KP_HASH]           = Q_KEY_CODE_KP_HASH,
    [SDL_SCANCODE_KP_SPACE]          = Q_KEY_CODE_KP_SPACE,
    [SDL_SCANCODE_KP_AT]             = Q_KEY_CODE_KP_AT,
    [SDL_SCANCODE_KP_EXCLAM]         = Q_KEY_CODE_KP_EXCLAM,
    [SDL_SCANCODE_KP_MEMSTORE]       = Q_KEY_CODE_KP_MEMSTORE,
    [SDL_SCANCODE_KP_MEMRECALL]      = Q_KEY_CODE_KP_MEMRECALL,
    [SDL_SCANCODE_KP_MEMCLEAR]       = Q_KEY_CODE_KP_MEMCLEAR,
    [SDL_SCANCODE_KP_MEMADD]         = Q_KEY_CODE_KP_MEMADD,
    [SDL_SCANCODE_KP_MEMSUBTRACT]    = Q_KEY_CODE_KP_MEMSUBTRACT,
    [SDL_SCANCODE_KP_MEMMULTIPLY]    = Q_KEY_CODE_KP_MEMMULTIPLY,
    [SDL_SCANCODE_KP_MEMDIVIDE]      = Q_KEY_CODE_KP_MEMDIVIDE,
    [SDL_SCANCODE_KP_PLUSMINUS]      = Q_KEY_CODE_KP_PLUSMINUS,
    [SDL_SCANCODE_KP_CLEAR]          = Q_KEY_CODE_KP_CLEAR,
    [SDL_SCANCODE_KP_CLEARENTRY]     = Q_KEY_CODE_KP_CLEARENTRY,
    [SDL_SCANCODE_KP_BINARY]         = Q_KEY_CODE_KP_BINARY,
    [SDL_SCANCODE_KP_OCTAL]          = Q_KEY_CODE_KP_OCTAL,
    [SDL_SCANCODE_KP_DECIMAL]        = Q_KEY_CODE_KP_DECIMAL,
    [SDL_SCANCODE_KP_HEXADECIMAL]    = Q_KEY_CODE_KP_HEXADECIMAL,
#endif
    [SDL_SCANCODE_LCTRL]             = Q_KEY_CODE_CTRL,
    [SDL_SCANCODE_LSHIFT]            = Q_KEY_CODE_SHIFT,
    [SDL_SCANCODE_LALT]              = Q_KEY_CODE_ALT,
    [SDL_SCANCODE_LGUI]              = Q_KEY_CODE_META_L,
    [SDL_SCANCODE_RCTRL]             = Q_KEY_CODE_CTRL_R,
    [SDL_SCANCODE_RSHIFT]            = Q_KEY_CODE_SHIFT_R,
    [SDL_SCANCODE_RALT]              = Q_KEY_CODE_ALTGR,
    [SDL_SCANCODE_RGUI]              = Q_KEY_CODE_META_R,
#if 0
    [SDL_SCANCODE_MODE]              = Q_KEY_CODE_MODE,
    [SDL_SCANCODE_AUDIONEXT]         = Q_KEY_CODE_AUDIONEXT,
    [SDL_SCANCODE_AUDIOPREV]         = Q_KEY_CODE_AUDIOPREV,
    [SDL_SCANCODE_AUDIOSTOP]         = Q_KEY_CODE_AUDIOSTOP,
    [SDL_SCANCODE_AUDIOPLAY]         = Q_KEY_CODE_AUDIOPLAY,
    [SDL_SCANCODE_AUDIOMUTE]         = Q_KEY_CODE_AUDIOMUTE,
    [SDL_SCANCODE_MEDIASELECT]       = Q_KEY_CODE_MEDIASELECT,
    [SDL_SCANCODE_WWW]               = Q_KEY_CODE_WWW,
    [SDL_SCANCODE_MAIL]              = Q_KEY_CODE_MAIL,
    [SDL_SCANCODE_CALCULATOR]        = Q_KEY_CODE_CALCULATOR,
    [SDL_SCANCODE_COMPUTER]          = Q_KEY_CODE_COMPUTER,
    [SDL_SCANCODE_AC_SEARCH]         = Q_KEY_CODE_AC_SEARCH,
    [SDL_SCANCODE_AC_HOME]           = Q_KEY_CODE_AC_HOME,
    [SDL_SCANCODE_AC_BACK]           = Q_KEY_CODE_AC_BACK,
    [SDL_SCANCODE_AC_FORWARD]        = Q_KEY_CODE_AC_FORWARD,
    [SDL_SCANCODE_AC_STOP]           = Q_KEY_CODE_AC_STOP,
    [SDL_SCANCODE_AC_REFRESH]        = Q_KEY_CODE_AC_REFRESH,
    [SDL_SCANCODE_AC_BOOKMARKS]      = Q_KEY_CODE_AC_BOOKMARKS,
    [SDL_SCANCODE_BRIGHTNESSDOWN]    = Q_KEY_CODE_BRIGHTNESSDOWN,
    [SDL_SCANCODE_BRIGHTNESSUP]      = Q_KEY_CODE_BRIGHTNESSUP,
    [SDL_SCANCODE_DISPLAYSWITCH]     = Q_KEY_CODE_DISPLAYSWITCH,
    [SDL_SCANCODE_KBDILLUMTOGGLE]    = Q_KEY_CODE_KBDILLUMTOGGLE,
    [SDL_SCANCODE_KBDILLUMDOWN]      = Q_KEY_CODE_KBDILLUMDOWN,
    [SDL_SCANCODE_KBDILLUMUP]        = Q_KEY_CODE_KBDILLUMUP,
    [SDL_SCANCODE_EJECT]             = Q_KEY_CODE_EJECT,
    [SDL_SCANCODE_SLEEP]             = Q_KEY_CODE_SLEEP,
    [SDL_SCANCODE_APP1]              = Q_KEY_CODE_APP1,
    [SDL_SCANCODE_APP2]              = Q_KEY_CODE_APP2,
#endif
};
