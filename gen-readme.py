
import json
import re

def print_vars_as_markdown():

  with open('help_variables.json', 'r', encoding='utf-8') as json_file:
    help_variables_data = json.load(json_file)

  sections = {'master': [], 'profile': [], 'rateprofile': []}

  doc_keys = 0
  for key in help_variables_data:
    assert help_variables_data[key]['section'] in sections
    # if help_variables_data[key]['section'] not in sections:
      # sections[help_variables_data[key]['section']] = []

    help_variables_data[key]['name'] = key
    sections[help_variables_data[key]['section']].append(help_variables_data[key])

    desc = help_variables_data[key]['desc']
    if desc != '' and not desc.upper().startswith('TODO') and not desc.upper().startswith('GUESS'):
      doc_keys += 1

  doc_percent = int(doc_keys / len(help_variables_data) * 100)

  # Print top of readme, without the list (generated by this script)
  with open('README.md', 'r', encoding='utf-8') as file:
    for line in file:
      if line.startswith("![Progress]"):
        print(f"![Progress](https://img.shields.io/badge/Documentation-{doc_percent}%25-blueviolet)")
      else:
        print(line, end = '')
      if line.startswith("<!-- BEGIN"):
        break

  for sec in sections:
    print(f"\n## Section `{sec}`")

    for helpdata in sections[sec]:
      print(f"\n### `{helpdata['name']}`")
      if 'title' in helpdata:
        print(f"> `{helpdata['title']}`")
      if 'default' in helpdata:
        print(f"- Default: `{helpdata['default']}`")
      if 'allowed' in helpdata:
        print(f"- Allowed: `{'`, `'.join(helpdata['allowed'])}`")
      if 'range' in helpdata:
        print(f"- Range: `{helpdata['range'][0]}` - `{helpdata['range'][1]}`")
      if 'unit' in helpdata:
        print(f"- Unit: `{helpdata['unit']}`")
      if 'datatype' in helpdata:
        print(f"- Type: `{helpdata['datatype']}`")
      if 'aka' in helpdata:
        print(f"- BF Configurator: *{helpdata['aka']}*")

      # 'desc' should always be there, but only print if not empty string
      if helpdata['desc']:
        # convert custom hash link syntax to markdown: #anchor -> [anchor](#ancher)
        desc = re.sub(r'(?<!\]\()#([\w_]+)', r'[\1](#\1)', helpdata['desc'])
        print(f"\n{desc}")

print_vars_as_markdown()
