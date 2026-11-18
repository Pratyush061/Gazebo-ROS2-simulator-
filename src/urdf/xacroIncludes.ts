/**
 * Resolves `<xacro:include filename="..."/>` against the in-browser workspace.
 *
 * The xacro processor evaluates a single string, so includes must be inlined
 * before compilation — exactly what real xacro does on disk. Supports relative
 * paths (`common/inertia.xacro`) and `$(find <package>)/...` substitution for
 * packages under `ros2_ws/src`.
 */
export interface XacroSourceFile {
  path: string;
  content: string;
}

export interface XacroIncludeDiagnostic {
  severity: 'error';
  path: string;
  message: string;
}

export interface XacroIncludeResult {
  xml: string;
  /** Absolute workspace paths that were inlined, in resolution order. */
  resolved: string[];
  diagnostics: XacroIncludeDiagnostic[];
}

/**
 * xacro include files are complete documents whose root is <robot>; only the
 * children are inlined, otherwise the parent <robot> would be nested.
 */
export function stripRobotWrapper(content: string): string {
  const open = content.match(/<robot\b[^>]*>/);
  if (!open) return content;
  const start = (open.index ?? 0) + open[0].length;
  const end = content.lastIndexOf('</robot>');
  if (end <= start) return content;
  return content.slice(start, end);
}

const INCLUDE_RE = /<xacro:include\s+filename\s*=\s*"([^"]+)"\s*\/?>(?:\s*<\/xacro:include>)?/g;
const MAX_DEPTH = 8;

function dirname(path: string): string {
  const idx = path.lastIndexOf('/');
  return idx === -1 ? '' : path.slice(0, idx);
}

/** Collapses `.` and `..` segments so include targets compare as plain paths. */
function normalise(path: string): string {
  const out: string[] = [];
  for (const part of path.split('/')) {
    if (part === '' || part === '.') continue;
    if (part === '..') out.pop();
    else out.push(part);
  }
  return out.join('/');
}

/**
 * Turns an include filename into a workspace path.
 * `$(find my_robot_description)/urdf/x.xacro` -> `ros2_ws/src/my_robot_description/urdf/x.xacro`
 */
export function resolveIncludeTarget(filename: string, fromPath: string): string {
  const findMatch = filename.match(/^\$\(find\s+([\w-]+)\)\/(.+)$/);
  if (findMatch) return normalise(`ros2_ws/src/${findMatch[1]}/${findMatch[2]}`);
  if (filename.startsWith('/')) return normalise(filename.slice(1));
  return normalise(`${dirname(fromPath)}/${filename}`);
}

/**
 * Inlines every xacro include reachable from `xml`.
 * Missing files and cycles become diagnostics instead of throwing, so the
 * Problems panel can report them the same way as any other compile error.
 */
export function resolveXacroIncludes(
  xml: string,
  filePath: string,
  files: XacroSourceFile[],
): XacroIncludeResult {
  const byPath = new Map(files.map((f) => [normalise(f.path), f]));
  const resolved: string[] = [];
  const diagnostics: XacroIncludeDiagnostic[] = [];
  const seen = new Set<string>([normalise(filePath)]);

  const inline = (source: string, currentPath: string, depth: number): string => {
    if (depth > MAX_DEPTH) {
      diagnostics.push({
        severity: 'error',
        path: currentPath,
        message: `xacro include nesting deeper than ${MAX_DEPTH} levels — check for a cycle.`,
      });
      return source;
    }

    return source.replace(INCLUDE_RE, (match, filename: string) => {
      const target = resolveIncludeTarget(filename, currentPath);

      if (seen.has(target)) {
        diagnostics.push({
          severity: 'error',
          path: currentPath,
          message: `Circular xacro include of "${filename}" (${target}).`,
        });
        return `<!-- circular include: ${filename} -->`;
      }

      const file = byPath.get(target);
      if (!file) {
        diagnostics.push({
          severity: 'error',
          path: currentPath,
          message: `xacro include not found: "${filename}" resolved to ${target}.`,
        });
        return `<!-- missing include: ${filename} -->`;
      }

      seen.add(target);
      resolved.push(target);
      const body = inline(stripRobotWrapper(file.content), target, depth + 1);
      return `<!-- inlined from ${target} -->\n${body}`;
    });
  };

  return { xml: inline(xml, normalise(filePath), 0), resolved, diagnostics };
}
