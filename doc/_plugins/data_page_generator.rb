# coding: utf-8
# Generate pages from individual records in yml files
# (c) 2014-2016 Adolfo Villafiorita
# Distributed under the conditions of the MIT License

require 'active_support/core_ext/hash/deep_merge'
require 'json'
require_relative 'doxygen'

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

    def display_category(category)
      return @categories.include?(category)
    end

    def initialize(site, base, out, schemas, categories, links = {})
      @site = site
      @base = base
      @dir = '/'
      @name = out
      @categories = categories

      menu = {}
      categories.each{ |category|
        menu[category] = {}
        schemas[category].each { |name, schema|
          menu[category][name] = {
            "active" => false,
            "display" => schema["title"].split("::").drop(1).join("::")
          }
        }
      }

      self.process(@name)
      self.read_yaml(File.join(base, '_layouts'), "json.html")
      self.data['title'] = 'titles.json'
      self.data['menu'] = menu
      self.data['all_schemas'] = {}
      self.data['links'] = links
      schemas.each { |category, cat_schemas|
        if display_category(category)
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

    def resolveAllOf(schema)
      if schema.is_a?(Array)
        schema.each_index{ | index |
          schema[index] = resolveAllOf(schema[index])
        }
        return schema
      end
      if not schema.is_a?(Hash)
        return schema
      end
      if schema.has_key?("allOf")
        schema["allOf"].each_index{ | index |
          resolveAllOf(schema["allOf"][index])
          schema = schema.deep_merge(resolveAllOf(schema["allOf"][index]))
        }
        schema.delete("allOf");
      end
      schema.each{ | key, value |
        schema[key] = resolveAllOf(schema[key])
      }
    end

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
          has_default = false
          if parent[key_out].key?("default")
            has_default = true
            default = parent[key_out]["default"].dup()
          end
          if category != "definitions"
            # Merge with surrounding schema 
            parent[key_out] = resolveAllOf(site.data["schemas"][category][name])
            if parent[key_out].has_key?("title")
              parent[key_out]["REF"] = "#{category}.#{name}"
            end
          else
            parent[key_out] = root[category][name].dup()
          end
          if has_desc
            parent[key_out]["DESC"] = desc
          end
          if has_default
            parent[key_out]["default"] = default
          end
        else
          resolveRef(site, value, schema, key, root)
        end
      }
    end

    # Try to resolve Doxygen link based on the schema's title and the doxytag file
    # Fills schema.api:
    # - doxygen id if there a doxygen entry was found in the doxytag file
    # - empty otherwise
    def resolveDoxygen(schema, name)
      doxygenId = get_page(name)
      unless doxygenId.empty?
        schema["api"] = doxygenId
      end
    end

    def generate(site)
      site.data["schemas"].each { |category, schemas|
        if category != "common"
          schemas.each { |name, schema|
            resolveRef(site, schema)
            schema = resolveAllOf(schema)
            resolveDoxygen(schema, schema['title'])
            site.data["schemas"][category][name] = schema
          }
        end
      }
      # Write generated schemas to a temporary file (for debug purposes)
      # File.open('/tmp/mc-rtc-doc-json-schemas.json', 'w') { |file| file.write(JSON.pretty_generate(site.data["schemas"])) }
      # puts "Generated json schema has been saved to /tmp/mc-rtc-doc-json-schemas.json"
      default_categories = ["mc_control", "mc_rbdyn", "ConstraintSet", "MetaTask", "State", "Observers"]
      site.pages << AllSchemasPage.new(site, site.source, 'json.html', site.data["schemas"], default_categories, {"All objects" => 'json-full.html'})
      site.pages << AllSchemasPage.new(site, site.source, 'json-full.html', site.data["schemas"], ["Eigen", "SpaceVecAlg", "RBDyn", "Tasks", "GUI"] + default_categories)
    end
  end

end
