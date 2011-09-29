var NAVTREE =
[
  [ "Repetier-Firmware", "index.html", [
    [ "Repetier-Firmware for Arduino based RepRaps", "index.html", null ],
    [ "Related Pages", "pages.html", [
      [ "Repetier-protocol", "_repetier-protocol.html", null ],
      [ "SdFat Library", "_arduino.html", null ],
      [ "Deprecated List", "deprecated.html", null ]
    ] ],
    [ "Data Structures", "annotated.html", [
      [ "biosParmBlock", "structbios_parm_block.html", null ],
      [ "cache_t", "unioncache__t.html", null ],
      [ "CID", "struct_c_i_d.html", null ],
      [ "csd_t", "unioncsd__t.html", null ],
      [ "CSDV1", "struct_c_s_d_v1.html", null ],
      [ "CSDV2", "struct_c_s_d_v2.html", null ],
      [ "directoryEntry", "structdirectory_entry.html", null ],
      [ "Extruder", "struct_extruder.html", null ],
      [ "fat32BootSector", "structfat32_boot_sector.html", null ],
      [ "GCode", "struct_g_code.html", null ],
      [ "masterBootRecord", "structmaster_boot_record.html", null ],
      [ "partitionTable", "structpartition_table.html", null ],
      [ "pin_map_t", "structpin__map__t.html", null ],
      [ "PrinterState", "struct_printer_state.html", null ],
      [ "PrintLine", "struct_print_line.html", null ],
      [ "Sd2Card", "class_sd2_card.html", null ],
      [ "SdFile", "class_sd_file.html", null ],
      [ "SdVolume", "class_sd_volume.html", null ],
      [ "SerialOutput", "class_serial_output.html", null ]
    ] ],
    [ "Data Structure Index", "classes.html", null ],
    [ "Data Fields", "functions.html", null ],
    [ "File List", "files.html", [
      [ "Repetier/Commands.cpp", "_commands_8cpp.html", null ],
      [ "Repetier/Configuration.h", "_configuration_8h.html", null ],
      [ "Repetier/Eeprom.cpp", "_eeprom_8cpp.html", null ],
      [ "Repetier/Eeprom.h", "_eeprom_8h.html", null ],
      [ "Repetier/Extruder.cpp", "_extruder_8cpp.html", null ],
      [ "Repetier/fastio.h", "fastio_8h.html", null ],
      [ "Repetier/FatStructs.h", "_fat_structs_8h.html", null ],
      [ "Repetier/gcode.cpp", "gcode_8cpp.html", null ],
      [ "Repetier/gcode.h", "gcode_8h.html", null ],
      [ "Repetier/pins.h", "pins_8h.html", null ],
      [ "Repetier/Repetier.pde", "_repetier_8pde.html", null ],
      [ "Repetier/Reptier.h", "_reptier_8h.html", null ],
      [ "Repetier/Sd2Card.cpp", "_sd2_card_8cpp.html", null ],
      [ "Repetier/Sd2Card.h", "_sd2_card_8h.html", null ],
      [ "Repetier/Sd2PinMap.h", "_sd2_pin_map_8h.html", null ],
      [ "Repetier/SdFat.h", "_sd_fat_8h.html", null ],
      [ "Repetier/SdFatmainpage.h", "_sd_fatmainpage_8h.html", null ],
      [ "Repetier/SdFatUtil.h", "_sd_fat_util_8h.html", null ],
      [ "Repetier/SdFile.cpp", "_sd_file_8cpp.html", null ],
      [ "Repetier/SdInfo.h", "_sd_info_8h.html", null ],
      [ "Repetier/SdVolume.cpp", "_sd_volume_8cpp.html", null ]
    ] ],
    [ "Globals", "globals.html", null ]
  ] ]
];

function createIndent(o,domNode,node,level)
{
  if (node.parentNode && node.parentNode.parentNode)
  {
    createIndent(o,domNode,node.parentNode,level+1);
  }
  var imgNode = document.createElement("img");
  if (level==0 && node.childrenData)
  {
    node.plus_img = imgNode;
    node.expandToggle = document.createElement("a");
    node.expandToggle.href = "javascript:void(0)";
    node.expandToggle.onclick = function() 
    {
      if (node.expanded) 
      {
        $(node.getChildrenUL()).slideUp("fast");
        if (node.isLast)
        {
          node.plus_img.src = node.relpath+"ftv2plastnode.png";
        }
        else
        {
          node.plus_img.src = node.relpath+"ftv2pnode.png";
        }
        node.expanded = false;
      } 
      else 
      {
        expandNode(o, node, false);
      }
    }
    node.expandToggle.appendChild(imgNode);
    domNode.appendChild(node.expandToggle);
  }
  else
  {
    domNode.appendChild(imgNode);
  }
  if (level==0)
  {
    if (node.isLast)
    {
      if (node.childrenData)
      {
        imgNode.src = node.relpath+"ftv2plastnode.png";
      }
      else
      {
        imgNode.src = node.relpath+"ftv2lastnode.png";
        domNode.appendChild(imgNode);
      }
    }
    else
    {
      if (node.childrenData)
      {
        imgNode.src = node.relpath+"ftv2pnode.png";
      }
      else
      {
        imgNode.src = node.relpath+"ftv2node.png";
        domNode.appendChild(imgNode);
      }
    }
  }
  else
  {
    if (node.isLast)
    {
      imgNode.src = node.relpath+"ftv2blank.png";
    }
    else
    {
      imgNode.src = node.relpath+"ftv2vertline.png";
    }
  }
  imgNode.border = "0";
}

function newNode(o, po, text, link, childrenData, lastNode)
{
  var node = new Object();
  node.children = Array();
  node.childrenData = childrenData;
  node.depth = po.depth + 1;
  node.relpath = po.relpath;
  node.isLast = lastNode;

  node.li = document.createElement("li");
  po.getChildrenUL().appendChild(node.li);
  node.parentNode = po;

  node.itemDiv = document.createElement("div");
  node.itemDiv.className = "item";

  node.labelSpan = document.createElement("span");
  node.labelSpan.className = "label";

  createIndent(o,node.itemDiv,node,0);
  node.itemDiv.appendChild(node.labelSpan);
  node.li.appendChild(node.itemDiv);

  var a = document.createElement("a");
  node.labelSpan.appendChild(a);
  node.label = document.createTextNode(text);
  a.appendChild(node.label);
  if (link) 
  {
    a.href = node.relpath+link;
  } 
  else 
  {
    if (childrenData != null) 
    {
      a.className = "nolink";
      a.href = "javascript:void(0)";
      a.onclick = node.expandToggle.onclick;
      node.expanded = false;
    }
  }

  node.childrenUL = null;
  node.getChildrenUL = function() 
  {
    if (!node.childrenUL) 
    {
      node.childrenUL = document.createElement("ul");
      node.childrenUL.className = "children_ul";
      node.childrenUL.style.display = "none";
      node.li.appendChild(node.childrenUL);
    }
    return node.childrenUL;
  };

  return node;
}

function showRoot()
{
  var headerHeight = $("#top").height();
  var footerHeight = $("#nav-path").height();
  var windowHeight = $(window).height() - headerHeight - footerHeight;
  navtree.scrollTo('#selected',0,{offset:-windowHeight/2});
}

function expandNode(o, node, imm)
{
  if (node.childrenData && !node.expanded) 
  {
    if (!node.childrenVisited) 
    {
      getNode(o, node);
    }
    if (imm)
    {
      $(node.getChildrenUL()).show();
    } 
    else 
    {
      $(node.getChildrenUL()).slideDown("fast",showRoot);
    }
    if (node.isLast)
    {
      node.plus_img.src = node.relpath+"ftv2mlastnode.png";
    }
    else
    {
      node.plus_img.src = node.relpath+"ftv2mnode.png";
    }
    node.expanded = true;
  }
}

function getNode(o, po)
{
  po.childrenVisited = true;
  var l = po.childrenData.length-1;
  for (var i in po.childrenData) 
  {
    var nodeData = po.childrenData[i];
    po.children[i] = newNode(o, po, nodeData[0], nodeData[1], nodeData[2],
        i==l);
  }
}

function findNavTreePage(url, data)
{
  var nodes = data;
  var result = null;
  for (var i in nodes) 
  {
    var d = nodes[i];
    if (d[1] == url) 
    {
      return new Array(i);
    }
    else if (d[2] != null) // array of children
    {
      result = findNavTreePage(url, d[2]);
      if (result != null) 
      {
        return (new Array(i).concat(result));
      }
    }
  }
  return null;
}

function initNavTree(toroot,relpath)
{
  var o = new Object();
  o.toroot = toroot;
  o.node = new Object();
  o.node.li = document.getElementById("nav-tree-contents");
  o.node.childrenData = NAVTREE;
  o.node.children = new Array();
  o.node.childrenUL = document.createElement("ul");
  o.node.getChildrenUL = function() { return o.node.childrenUL; };
  o.node.li.appendChild(o.node.childrenUL);
  o.node.depth = 0;
  o.node.relpath = relpath;

  getNode(o, o.node);

  o.breadcrumbs = findNavTreePage(toroot, NAVTREE);
  if (o.breadcrumbs == null)
  {
    o.breadcrumbs = findNavTreePage("index.html",NAVTREE);
  }
  if (o.breadcrumbs != null && o.breadcrumbs.length>0)
  {
    var p = o.node;
    for (var i in o.breadcrumbs) 
    {
      var j = o.breadcrumbs[i];
      p = p.children[j];
      expandNode(o,p,true);
    }
    p.itemDiv.className = p.itemDiv.className + " selected";
    p.itemDiv.id = "selected";
    $(window).load(showRoot);
  }
}

