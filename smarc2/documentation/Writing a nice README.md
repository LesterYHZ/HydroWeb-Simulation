# Writing a _nice_ README
> nice: giving pleasure or satisfaction; pleasant or attractive.

Between pleasure and satisfaction, satisfaction should be the goal here. 
And between pleasant and attractive, attractive gets the cake.


## Satisfaction
> satisfaction: fulfilment of one's wishes, expectations, or needs

Why do people read a README?
Because they _need_ something, and they need to learn how to _get that thing_.

- Wish: the thing, ASAP
- Need: the method of procurement
- Expectation: learned the method to get the thing

Thus:

- Keep the readme short.
  - Link to further reading for minute details.
  - Most people "just want to run" the thing and don't care _how_ it runs.
- List the dependencies and prior steps **clearly**.
  - Header with "STEP ONE"?
- Use bullets for lists of things to do (`- text` in markdown, leave one space above and below the points)
  - Indentation is as useful here as it is in code (two spaces before `-` for each level)
- Use images if that would be quicker.
  -  (`![image-title](../documentation/media/image.png)` will show an image as part of the markdown. This example uses a relative path.)
  - If you're talking about a graph structure, _showing_ it is always better than describing it.
- Provide copy-pastable examples (see the previous bullets)
  - Use codeblocks to make it obvious that this can be copied (` `` `)

## Attractive
> attractive: having qualities or features which arouse interest.

- Interest: you already have someone interested in _the thing_ your repo has.
  - Make them interested in reading its README too!
  - Put in entire examples in it. If someone can use your README as a manual to use your thing, they will keep coming back to it and actually read it.
- Use images and bullets! (see above).
- Give it a big title.
- Use **bold** boldly.
- Split it up into headings (`# H1`, `## H2`... ).
- Use quotes for "the point" (`> text` leave one empty line after to finis).