# AiBoids: Reynolds-style BOIDS, Autonomous Steering Behaviours, and UML State Machines in Python

![Sheep/Dog Flocking Demo](https://github.com/PigSupreme/aiboids/blob/58d9cb2f99bd24d77b9e422e2e40b31e7093807a/docs/sheep_demo.gif)

**Not-so-tragic backstory**: In the aftermath of the world NOT ending in 2012, I found myself with some free time and finally committed to learning Python. Having taught myself programming sometime during the Dark Ages (MS-BASIC and TurboPascal...thar be dragons!), I tend to find introductory tutorials rather boring. This was my attempt to do something more challenging and engaging. Fast-forward a few years, add GitHub for repository management, and here we are.

This work was inspired by and draws heavily from:
* [Craig Reynolds BOIDS Page](http://www.red3d.com/cwr/boids/).
* [*Steering Behaviours for Autonomous Characters*](http://www.red3d.com/cwr/steer/), also by Craig Reynolds.
* [Programming Game AI by Example](http://www.ai-junkie.com/books/toc_pgaibe.html) by Mat Buckland.

## Technology
* [Python 3.8](https://www.python.org/).
* [Pygame 2](https://www.pygame.org/wiki/about) for real-time 2D rendering.
* [Sphinx](https://www.sphinx-doc.org/) for automated documentation.
* [Hypothesis](https://hypothesis.readthedocs.io/en/latest/) for unit-testing all of the math.

It's worth noting that [NumPy](https://numpy.org/) or similar math packages are NOT required; this is a holdover from some related projects on the RaspberyPi where storage space was at a premium, and from some student projects for non-programmers.

## How to Use This?
Clone the repository to your local machine; setup and dependencies were designed to be minimal. The WestWorld3 demo is console-based; others require Pygame somewhere in your PYTHONPATH. All demos can be run from the command line, for example:

```$ python3 {path-to-local-repo}/demos/sheep.py```

---

Further information and documentation: https://aiboids.readthedocs.io/en/latest/
