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
      if schema and schema.key?('title')
        self.data['title']  = 'Schema documentation for ' + schema['title']
      else
        self.data['title'] = 'Schema documentation'
      end
      self.data['schema'] = schema
      self.data['menu'] = Marshal.load(Marshal.dump(menu))
      self.data['category'] = category
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
            menu[category][name] = {}
            menu[category][name]["active"] = false
            menu[category][name]["display"] = schema["title"].split("::").drop(1).join("::")
          }
        end
      }
      site.data["schemas"].each { |category, schemas|
        if category != "common"
          schemas.each { |name, schema|
            menu[category][name]["active"] = true
            resolveRef(site, schema)
            site.pages << SchemaPage.new(site, site.source, File.join("schemas", category), name, schema, menu, category)
            menu[category][name]["active"] = false
          }
        end
      }
      site.pages << SchemaPage.new(site, site.source, "", "json", nil, menu, "")
    end
  end

end
