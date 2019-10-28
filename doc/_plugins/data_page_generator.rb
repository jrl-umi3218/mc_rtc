# coding: utf-8
# Generate pages from individual records in yml files
# (c) 2014-2016 Adolfo Villafiorita
# Distributed under the conditions of the MIT License

module Jekyll

  module Sanitizer
    # strip characters and whitespace to create valid filenames, also lowercase
    def sanitize_filename(name)
      if(name.is_a? Integer)
        return name.to_s
      end
      return name.tr(
  "ÀÁÂÃÄÅàáâãäåĀāĂăĄąÇçĆćĈĉĊċČčÐðĎďĐđÈÉÊËèéêëĒēĔĕĖėĘęĚěĜĝĞğĠġĢģĤĥĦħÌÍÎÏìíîïĨĩĪīĬĭĮįİıĴĵĶķĸĹĺĻļĽľĿŀŁłÑñŃńŅņŇňŉŊŋÑñÒÓÔÕÖØòóôõöøŌōŎŏŐőŔŕŖŗŘřŚśŜŝŞşŠšſŢţŤťŦŧÙÚÛÜùúûüŨũŪūŬŭŮůŰűŲųŴŵÝýÿŶŷŸŹźŻżŽž",
  "AAAAAAaaaaaaAaAaAaCcCcCcCcCcDdDdDdEEEEeeeeEeEeEeEeEeGgGgGgGgHhHhIIIIiiiiIiIiIiIiIiJjKkkLlLlLlLlLlNnNnNnNnnNnNnOOOOOOooooooOoOoOoRrRrRrSsSsSsSssTtTtTtUUUUuuuuUuUuUuUuUuUuWwYyyYyYZzZzZz"
).downcase.strip.gsub(' ', '-').gsub(/[^\w.-]/, '')
    end
  end

  # this class is used to tell Jekyll to generate a page
  class SchemaPage < Page
    include Sanitizer

    def initialize(site, base, dir, name, schema, menu, category)
      @site = site
      @base = base
      @dir  = dir
      @name = name + ".html"

      self.process(@name)
      self.read_yaml(File.join(base, '_layouts'), "json.html")
      self.data['title']  = 'Schema documentation for ' + schema['title']
      self.data['schema'] = schema
      self.data['menu'] = Marshal.load(Marshal.dump(menu))
      self.data['category'] = category
    end
  end

  class SchemaPagesGenerator < Generator
    safe true

    def resolveRef(site, schema)
      if schema.key?("properties")
        schema["properties"].each{ | prop, value |
          if value.key?("$ref")
            category = value["$ref"].split('/')[-2]
            name = value["$ref"].split('/')[-1].gsub(".json", "").gsub(".", "")
            resolveRef(site, site.data["schemas"][category][name])
            schema["properties"][prop] = site.data["schemas"][category][name]
            schema["properties"][prop]["REF"] = "#{category}/#{name}"
          end
        }
      end
    end

    def generate(site)
      menu = {}
      site.data["schemas"].each { |category, schemas|
        if category != "common"
          menu[category] = {}
          schemas.each { |name, schema|
            menu[category][name] = false
          }
        end
      }
      site.data["schemas"].each { |category, schemas|
        if category != "common"
          schemas.each { |name, schema|
            menu[category][name] = true
            resolveRef(site, schema)
            site.pages << SchemaPage.new(site, site.source, File.join("schemas", category), name, schema, menu, category)
            menu[category][name] = false
          }
        end
      }
    end
  end

end
