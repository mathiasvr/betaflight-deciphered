import json
import re


def parse_vars_and_print():

  with open('get_413_defaults.txt', 'r', encoding='utf-8') as file:
    lines = file.read().splitlines()


  with open('help_variables.json') as json_file:
      help_variables_data = json.load(json_file)
      # for p in data['people']:
      #     print('Name: ' + p['name'])
      #     print('Website: ' + p['website'])
      #     print('From: ' + p['from'])
      #     print('')

  # data = [int(i) for i in lines[0].split()]

  # remove empty lines (TODO: also remove whitespace?)
  lines = [line for line in lines if line != '']

  # for line in lines:
  i = 0
  while i < len(lines):
    matches = re.match("(.+) = (.+)", lines[i])
    if matches:
      name, default_value = matches.groups()

      i += 1

      # matches = re.match("(profile|rateprofile).+", lines[i])
      section = 'master'
        # todo: profile / rateprofile entry (skip for now)
      if lines[i].startswith('profile'):
        section = 'profile'
        i += 1
      elif lines[i].startswith('rateprofile'):
        section = 'rateprofile'
        i += 1

      matches = re.match("Allowed .+: (.+)", lines[i])
      allowed_values = "Unknown"
      if matches:
        allowed_values = matches.groups()[0]

      else:
        matches = re.match("(Array length.+)", lines[i])
        if matches:
          allowed_values = matches.groups()[0]
        else:
          # reparse entry
          i -= 1

      helpdata = help_variables_data[name]
      desc = helpdata['desc']
      aka = helpdata['aka'] if 'aka' in helpdata else ''

      print(f"| {name} | {aka} | {desc} | {default_value} | {allowed_values} |") #{section}")
      # print(f'\t{{ "{name}": "" }},')
    else:
      print("Parsing error!")
      print("Line:")
      print(lines[i])
      exit(1)

    i += 1


print("| Variable name | BF Configurator Name | Description | Default Value | Allowed Values |")
print("| ------ | ------ | ------ | ------ | ------ |")

parse_vars_and_print()
