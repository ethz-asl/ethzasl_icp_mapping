#!/usr/bin/env python

import yaml
import os.path
import sys

def dump_node(node, param_list=[], prefix=""):
	try:
		for key in node.keys():
			dump_node(node[key], param_list, prefix="%s/%s"%(prefix, key))
	except AttributeError:
		param_list.append("%s=%s"%(prefix, str(node)))


def yaml2txt(filename):
	yamlfile = open(filename)
	param_list = []
	document = yaml.load(yamlfile)
	yamlfile.close()
	dump_node(document, param_list)
	txtfile = open(os.path.splitext(filename)[0] + '.txt', 'w')
	txtfile.write(' '.join(param_list)+'\n')
	txtfile.close()


def main():
	for f in sys.argv[1:]:
		yaml2txt(f)


if __name__=="__main__":
	main()
