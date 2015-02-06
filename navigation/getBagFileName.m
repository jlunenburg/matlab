function bagfilename = getBagFileName(item);

global itemList
global filepath

citem = itemList{item};

bagfilename = strcat(filepath, citem.Attributes.stamp);
bagfilename = strcat(bagfilename, '.bag');