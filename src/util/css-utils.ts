type Classes = Partial<{ [key: string]: string }>;

export function className<T extends Classes>(
  classes: T | undefined,
  name: keyof T,
): string {
  if (!classes || !classes[name]) {
    return '';
  }
  return classes[name]!;
}
