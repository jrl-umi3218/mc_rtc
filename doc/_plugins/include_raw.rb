module Jekyll
  module Tags
    class RawIncludeTag < IncludeTag
      def read_file(file, context)
        # Wrap the entire file in a `raw` liquid tag, suppressing liquid processing.
        "{% raw %}" + super + "{% endraw %}"
      end
    end
  end
end

Liquid::Template.register_tag("include_raw", Jekyll::Tags::RawIncludeTag)
