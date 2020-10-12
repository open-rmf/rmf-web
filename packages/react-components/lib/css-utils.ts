export function joinClasses(...classes: (string | undefined)[]): string {
  return classes.filter((value) => value).join(' ');
}
