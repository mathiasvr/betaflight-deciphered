# parse output from betaflight cli 'get' command (variables, default values, etc.) to json.
import json
import re

with open('get_413_defaults.txt', 'r', encoding='utf-8') as file:
  lines = file.read().splitlines()

with open('help_variables.json', 'r', encoding='utf-8') as json_file:
  help_variables_data = json.load(json_file)

# remove empty lines (TODO: also remove whitespace?)
lines = [line for line in lines if line != '']

json_variables = {}

i = 0
while i < len(lines):
  matches = re.match("(.+) = (.+)", lines[i])
  if matches:
    name, default_value = matches.groups()

    # Only keep 'desc' and 'aka'. The rest is parsed
    json_variables[name] = { 'desc': help_variables_data[name]['desc'] }
    if 'aka' in help_variables_data[name]:
       json_variables[name]['aka'] = help_variables_data[name]['aka']

    if default_value != '-':
      json_variables[name]['default'] = default_value

    i += 1

    section = 'master'
    if lines[i].startswith('profile'):
      section = 'profile'
      i += 1
    elif lines[i].startswith('rateprofile'):
      section = 'rateprofile'
      i += 1

    matches = re.match("Allowed (.+): (.+)", lines[i])

    if matches:
      if matches.groups()[0] == 'values':
        json_variables[name]['allowed'] = [s.strip() for s in matches.groups()[1].split(',')]
      else: # 'range'
         json_variables[name]['range'] = [int(s) for s in matches.groups()[1].split(' - ')]
         assert len(json_variables[name]['range']) == 2

    else:
      matches = re.match("Array length: (.+)", lines[i])
      if matches:
        json_variables[name]['datatype'] = f"Array[{matches.groups()[0]}]"
      else:
        # reparse entry
        i -= 1

    assert section
    json_variables[name]['section'] = section

  else:
    print("Parsing error!")
    print("Line:")
    print(lines[i])
    exit(1)

  i += 1

with open('help_variables.json', 'w', encoding='utf-8') as json_file:
  json.dump(json_variables, json_file, indent=2) 
