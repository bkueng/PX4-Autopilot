from xml.sax.saxutils import escape
import codecs
import json
import gzip #to create .gz file of json
import sys


class JsonOutput():
    def __init__(self, groups, board, inject_xml_file_name):
        all_json=dict()
        all_json['version']=1
        all_json['uid']=1
        all_json['scope']="Firmware"
        all_params=[]
        all_json['parameters']=all_params

        #xml_parameters = ET.Element("parameters")
        #xml_version = ET.SubElement(xml_parameters, "version")
        #xml_version.text = "3"
        #xml_version = ET.SubElement(xml_parameters, "parameter_version_major")
        #xml_version.text = "1"
        #xml_version = ET.SubElement(xml_parameters, "parameter_version_minor")
        #xml_version.text = "15"

        #importtree = ET.parse(inject_xml_file_name)
        #injectgroups = importtree.getroot().findall("group")
        #for igroup in injectgroups:
        #    xml_parameters.append(igroup)

        schema_map = {
                        "short_desc": "shortDescription",
			"long_desc": "longDescription",
			"unit": "units",
			"decimal": "decimalPlaces",
			"min": "minValue",
			"max": "maxValue",
			"increment": "increment",
			"reboot_required": "rebootRequired"
			}

        last_param_name = ""
        board_specific_param_set = False
        for group in groups:
            group_name=group.GetName()

            for param in group.GetParams():
                if (last_param_name == param.GetName() and not board_specific_param_set) or last_param_name != param.GetName():
                    curr_param=dict()
                    curr_param['name'] = param.GetName()
                    curr_param['defaultValue'] = param.GetDefault()
                    curr_param['type'] = param.GetType().capitalize()
                    curr_param['group'] = group_name
                    if (param.GetCategory()):
                        curr_param['category'] = param.GetCategory()

                    if (param.GetVolatile() == "true"):
                        curr_param['volatile'] = param.GetVolatile()

                    last_param_name = param.GetName()
                    for code in param.GetFieldCodes():
                        value = param.GetFieldValue(code)
                        if code == "board":
                            if value == board:
                                board_specific_param_set = True
                                # JSON schema has no field for board_specific schema. Ignore.
                            else:
                                #xml_group.remove(xml_param)
                                continue
                        else:
                            #map PX4 param field names to schema names
                           if code in schema_map:
                               curr_param[schema_map[code]] = value
                           else:
                               print('ERROR: Field not in json schema: %s' % code)
                               sys.exit(1)


                if last_param_name != param.GetName():
                    board_specific_param_set = False

                enum_codes=param.GetEnumCodes() or '' # Gets numerical values for parameter.
                if enum_codes:
                    enum_codes=sorted(enum_codes,key=float)
                    codes_list=list()
                    for item in enum_codes:
                        code_dict=dict()
                        code_dict['value']=item
                        code_dict['description']=param.GetEnumValue(item)
                        codes_list.append(code_dict)
                    curr_param['values'] = codes_list


                if len(param.GetBitmaskList()) > 0:
                    bitmasks_list=list()
                    for index in param.GetBitmaskList():
                        bitmask_dict=dict()
                        bitmask_dict['index']=index
                        bitmask_dict['description']=param.GetBitmaskBit(index)
                        bitmasks_list.append(bitmask_dict)
                    curr_param['bitmask'] = bitmasks_list


                all_params.append(curr_param)

 


        '''
        result = ""
        all_json=dict()
        all_json['version']=1
        all_json['uid']=1
        all_json['scope']="Firmware"
        all_params=[]
        all_json['parameters']=all_params
        for group in groups:
            #result += '## %s\n\n' % group.GetName()
            group_name=group.GetName()
            for param in group.GetParams():
                curr_param=dict()
                curr_param['name'] = param.GetName()
                curr_param['type'] = param.GetType().capitalize()
                curr_param['group'] = group_name
                curr_param['category'] = param.GetCategory()
                curr_param['shortDescription'] = param.GetFieldValue("short_desc")
                curr_param['longDescription'] = param.GetFieldValue("long_desc")
                curr_param['units'] = param.GetFieldValue("unit")
                curr_param['defaultValue'] = param.GetDefault()

                curr_param['decimalPlaces'] = param.GetFieldValue("decimal")
                curr_param['minValue'] = param.GetFieldValue("min")
                curr_param['maxValue'] = param.GetFieldValue("max")
                curr_param['increment'] = param.GetFieldValue("increment")
                curr_param['rebootRequired'] = param.GetFieldValue("reboot_required")
                curr_param['volatile'] = param.GetVolatile()





                all_params.append(curr_param)
        '''


        #Note clear if we need additionalProperties, required, and what to do if values not defined.

        #Json string output.
        self.output = json.dumps(all_json,indent=2)




    def Save(self, filename):
        with codecs.open(filename, 'w', 'utf-8') as f:
            f.write(self.output)

        #create gz version
        gz_filename=filename+'.gz'
        with gzip.open(gz_filename, 'wt') as f:
            f.write(self.output)
