
import json

def print_vars_as_markdown():

  with open('help_variables.json', 'r', encoding='utf-8') as json_file:
    help_variables_data = json.load(json_file)

  sections = {'master': [], 'profile': [], 'rateprofile': []}

  for key in help_variables_data:
    assert help_variables_data[key]['section'] in sections
    # if help_variables_data[key]['section'] not in sections:
      # sections[help_variables_data[key]['section']] = []

    help_variables_data[key]['name'] = key 
    sections[help_variables_data[key]['section']].append(help_variables_data[key])

    
  for sec in sections:
    print(f"\n## Section `{sec}`")
    
    for helpdata in sections[sec]:

  # for key in help_variables_data:
  #   helpdata = help_variables_data[key]

      print(f"\n### `{helpdata['name']}`")
      if 'default' in helpdata:
        print(f"- Default: `{helpdata['default']}`")
      if 'allowed' in helpdata:
        print(f"- Allowed: `{'`, `'.join(helpdata['allowed'])}`")
      if 'range' in helpdata:
        print(f"- Range: `{helpdata['range'][0]}` - `{helpdata['range'][1]}`")
      if 'datatype' in helpdata:
        print(f"- Type: `{helpdata['datatype']}`")
      if 'aka' in helpdata:
        print(f"- BF Configurator: *{helpdata['aka']}*")

      # 'desc' should always be there, but only print if not empty string
      if helpdata['desc']:
        print(f"\n{helpdata['desc']}")


# Print top of readme, without the list (generated by this script)
with open('README.md', 'r', encoding='utf-8') as file:
  # lines = file.read().splitlines()
  for line in file:
    print(line, end = '')
    if line.startswith("<!-- BEGIN"):
      break

print_vars_as_markdown()
