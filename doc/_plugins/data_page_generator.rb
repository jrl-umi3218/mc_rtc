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
      categories.each do |category|
        menu[category] = {}
        schemas[category].each do |name, schema|
          # one can include the full namespace API
          # in the title to automatically link it with the corresponding doxygen page
          # Strip the namespace for the menu display
          display = schema["title"].split("::").last
          menu[category][name] = {
            "active" => false,
            "display" => display
          }
        end
      end

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
            self.data['all_schemas'][category][name]["schema"] = schema

            example_json = {}
            example_json["lang"] = "json"
            example_json["name"] = "JSON"
            json_path = File.join(base, '_examples', 'json', category, name + '.json')
            begin
              example_json["source"] = File.read(json_path)
            rescue Errno::ENOENT
              msg = "ERROR: JSON example file not found. Please create: #{json_path}"
              example_json["source"] = "//#{msg}"
              puts msg
            end

            example_yaml = {}
            example_yaml["lang"] = "yaml"
            example_yaml["name"] = "YAML"
            yaml_path = File.join(base, '_examples', 'yaml', category, name + '.yaml')
            begin
              example_yaml["source"] = File.read(yaml_path)
            rescue Errno::ENOENT
              msg = "ERROR: YAML example file not found. Please create: #{yaml_path}"
              example_yaml["source"] = "# #{msg}"
              puts msg
            end

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



    # FIXME::
    # - does not resolve oneOf within "array": { "item": { ... } }
    # - resolve additionalProperties
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
          # Recursively resolve the referenced schema
          if category != "definitions"
            if site.data["schemas"].key?(category) && site.data["schemas"][category].key?(name)
              resolveRef(site, site.data["schemas"][category][name])
              referenced = site.data["schemas"][category][name].dup()
            else
              puts "[resolveRef] WARNING: Path does not exist: site.data['schemas'][#{category}][#{name}]"
              referenced = {}
            end
          else
            if root.is_a?(Hash) && root.key?(category) && root[category].is_a?(Hash) && root[category].key?(name)
              resolveRef(site, root[category][name], nil, nil, root)
              referenced = root[category][name].dup()
            else
              puts "[resolveRef] WARNING: Path does not exist: root['#{category}']['#{name}']"
              referenced = {}
            end
          end

          # Remove $ref from the referencing object before merging
          referencing = parent[key_out].dup
          referencing.delete("$ref")

          # Merge referenced schema and referencing object, referencing takes precedence
          merged = referenced.deep_merge(referencing)

          parent[key_out] = merged
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
      File.open('/tmp/mc-rtc-doc-json-schemas.json', 'w') { |file| file.write(JSON.pretty_generate(site.data["schemas"])) }
      # puts "Generated json schema has been saved to /tmp/mc-rtc-doc-json-schemas.json"
      default_categories = ["mc_rtc", "mc_control", "mc_rbdyn", "ConstraintSet", "MetaTask", "State", "Observers"]
      site.pages << AllSchemasPage.new(site, site.source, 'json.html', site.data["schemas"], default_categories, {"All objects" => 'json-full.html'})
      site.pages << AllSchemasPage.new(site, site.source, 'json-full.html', site.data["schemas"], ["Eigen", "SpaceVecAlg", "RBDyn", "Tasks", "GUI"] + default_categories)
    end
  end

end
