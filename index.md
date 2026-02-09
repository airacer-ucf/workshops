---
layout: default
title: Workshops
---

# Workshops

<ul>
  {% assign files = site.static_files | where_exp: "f", "f.path contains page.dir" %}
  {% for f in files %}
    {% unless f.path == page.path %}
      <li><a href="{{ f.path | relative_url }}">{{ f.name }}</a></li>
    {% endunless %}
  {% endfor %}
</ul>

