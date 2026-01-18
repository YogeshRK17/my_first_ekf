fetch("tree.xml")
  .then(r => r.text())
  .then(text => {
    const xml = new DOMParser().parseFromString(text, "text/xml");
    const bt = xml.querySelector("BehaviorTree");
    const root = buildTree(bt.children[0]);
    layoutTree(root, 600, 50);
    drawTree(root);
  });

/* ---------------- TREE MODEL ---------------- */

function buildTree(xmlNode) {
  return {
    type: xmlNode.tagName,
    name: xmlNode.getAttribute("name") || "",
    children: Array.from(xmlNode.children).map(buildTree),
    x: 0,
    y: 0
  };
}

/* ---------------- LAYOUT ---------------- */

const LEVEL_Y = 120;      // was 90
const NODE_SPACING = 220; // was 120


function layoutTree(node, x, y) {
  node.y = y;

  if (node.children.length === 0) {
    node.x = x;
    return NODE_SPACING;
  }

  let width = 0;
  let startX = x - ((node.children.length - 1) * NODE_SPACING) / 2;

  node.children.forEach((child, i) => {
    width += layoutTree(child, startX + i * NODE_SPACING, y + LEVEL_Y);
  });

  node.x =
    (node.children[0].x +
      node.children[node.children.length - 1].x) / 2;

  return width;
}

/* ---------------- DRAW ---------------- */

function drawTree(node) {
  const svg = document.getElementById("tree");

  node.children.forEach(child => {
    drawLine(svg, node.x, node.y, child.x, child.y);
    drawTree(child);
  });

  drawNode(svg, node.x, node.y, node);
}

function drawNode(svg, x, y, node) {
  const g = document.createElementNS("http://www.w3.org/2000/svg", "g");

  const rectWidth = 180;
  const rectHeight = 50;

  const rect = document.createElementNS(svg.namespaceURI, "rect");
  rect.setAttribute("x", x - rectWidth / 2);
  rect.setAttribute("y", y - rectHeight / 2);
  rect.setAttribute("width", rectWidth);
  rect.setAttribute("height", rectHeight);
  rect.setAttribute("class", "node");

  const text = document.createElementNS(svg.namespaceURI, "text");
  text.setAttribute("x", x);
  text.setAttribute("y", y);
  text.setAttribute("class", "text");
  text.textContent = `${node.type}${node.name ? ":" + node.name : ""}`;

  g.appendChild(rect);
  g.appendChild(text);
  svg.appendChild(g);
}


function drawLine(svg, x1, y1, x2, y2) {
  const line = document.createElementNS(svg.namespaceURI, "line");
  line.setAttribute("x1", x1);
  line.setAttribute("y1", y1 + 25);
  line.setAttribute("x2", x2);
  line.setAttribute("y2", y2 - 25);
  line.setAttribute("class", "line");
  svg.appendChild(line);
}
