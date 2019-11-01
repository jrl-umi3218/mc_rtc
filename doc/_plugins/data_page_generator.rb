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

  class AllSchemasPage < Page
    include Sanitizer

    def initialize(site, base, menu, schemas)
      @site = site
      @base = base
      @dir = '/'
      @name = 'json.html'

      self.process(@name)
      self.read_yaml(File.join(base, '_layouts'), "json.html")
      self.data['title'] = 'Schema documentation'
      self.data['menu'] = menu
      self.data['all_schemas'] = {}
      schemas.each { |category, cat_schemas|
        if category != "common"
          self.data['all_schemas'][category] = {}
          cat_schemas.each { |name, schema|
            self.data['all_schemas'][category][name] = {}
            self.data['all_schemas'][category][name]["schema"] = schema;
            example_json = {}
            example_json["lang"] = "json"
            example_json["name"] = "JSON"
            example_json["source"] = File.read(File.join(base, '_examples', 'json', category, name + '.json'))
            example_yaml = {}
            example_yaml["lang"] = "yaml"
            example_yaml["name"] = "YAML"
            example_yaml["source"] = File.read(File.join(base, '_examples', 'yaml', category, name + '.yaml'))
            self.data['all_schemas'][category][name]["example"] = [example_json, example_yaml]
          }
        end
      }
    end
  end


  class SchemaPagesGenerator < Generator
    safe true

    def resolveRef(site, schema, parent = nil, key_out = nil, root = nil)
      if root == nil
        root = schema
      end
      if not schema.is_a?(Hash)
        if schema.is_a?(Array)
          schema.each_index{ | index |
            resolveRef(site, schema[index], schema, index, root)
          }
        end
        return
      end
      schema.each{ | key, value |
        if key == "$ref"
          category = value.split('/')[-2]
          name = value.split('/')[-1].gsub(".json", "").gsub(".", "")
          if category != "definitions"
            resolveRef(site, site.data["schemas"][category][name])
          else
            resolveRef(site, root[category][name], nil, nil, root)
          end
          has_desc = false
          if parent[key_out].key?("description")
            has_desc = true
            desc = parent[key_out]["description"].dup()
          end
          if category != "definitions"
            parent[key_out] = site.data["schemas"][category][name].dup()
            parent[key_out]["REF"] = "#{category}.#{name}"
          else
            parent[key_out] = root[category][name].dup()
          end
          if has_desc
            parent[key_out]["DESC"] = desc
          end
        else
          resolveRef(site, value, schema, key, root)
        end
      }
    end

    def generate(site)
      menu = {}
      default_order = ["Eigen", "SpaceVecAlg", "RBDyn", "Tasks", "mc_rbdyn_urdf", "mc_rbdyn", "ConstraintSet", "MetaTask"]
      default_order.each { |name|
        menu[name] = {}
      }
      site.data["schemas"].each { |category, schemas|
        if category != "common"
          menu[category] = {}
          schemas.each { |name, schema|
            resolveRef(site, schema)
            menu[category][name] = {}
            menu[category][name]["active"] = false
            menu[category][name]["display"] = schema["title"].split("::").drop(1).join("::")
          }
        end
      }
      site.pages << AllSchemasPage.new(site, site.source, menu, site.data["schemas"])
    end
  end

end
