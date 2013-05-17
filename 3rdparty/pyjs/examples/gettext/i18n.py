from pyjamas.JSONTranslations import JSONTranslations

__all__ = ['i18n', 'gettext', 'ngettext', '_']


class Translations(JSONTranslations):
    base_url = "lang"
    domain = "Gettext"

i18n = Translations()
gettext = i18n.gettext
ngettext = i18n.ngettext
_ = gettext
