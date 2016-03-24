#ifndef QEMU_HELP_OPTION_H
#define QEMU_HELP_OPTION_H 1

/**
 * is_help_option:
 * @s: string to test
 *
 * Check whether @s is one of the standard strings which indicate
 * that the user is asking for a list of the valid values for a
 * command option like -cpu or -M. The current accepted strings
 * are 'help' and '?'. '?' is deprecated (it is a shell wildcard
 * which makes it annoying to use in a reliable way) but provided
 * for backwards compatibility.
 *
 * Returns: true if @s is a request for a list.
 */
static inline bool is_help_option(const char *s)
{
    return !strcmp(s, "?") || !strcmp(s, "help");
}

#endif
