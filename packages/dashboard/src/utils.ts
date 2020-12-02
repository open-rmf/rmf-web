export function mergeContent<T extends Record<string, unknown>>(
  currentContent: T,
  storedContent: T,
  replaceCurrentContent = true,
): T {
  // In general, the constraint Record<string, XXX> doesn't actually ensure that an argument has a
  // string index signature, it merely ensures that the properties of the argument are assignable
  // to type XXX. So, in the example above you could effectively pass any object and the function
  //could write to any property without any checks. https://github.com/microsoft/TypeScript/issues/31661
  const newContents = Object.assign({}, currentContent) as Record<string, unknown>;
  Object.keys(storedContent).forEach((element: string) => {
    if (replaceCurrentContent) newContents[element] = storedContent[element];
    else {
      // If the element is already on the currentContent do not replace it
      if (!(element in currentContent)) newContents[element] = storedContent[element];
    }
  });
  return newContents as T;
}
