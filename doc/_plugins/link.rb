module Jekyll
  class LinkTag < Liquid::Tag

    def initialize(tag_name, text, tokens)
      super
      @text = text.strip()
    end

    def render(context)
      url = context['site']['data']['links'][@text]['url']
      "<a href=\"#{url}\" target=\"blank_\">#{@text}</a>"
    end
  end
end

Liquid::Template.register_tag('link', Jekyll::LinkTag)
