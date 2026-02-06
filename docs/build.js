import fs from 'node:fs';
import path from 'node:path';
import ts from 'typescript';

// find all .ts files in src
const srcDir = path.join(path.dirname(new URL(import.meta.url).pathname), '../src');

function getAllSourceFiles(dir) {
    let files = [];
    for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
        const fullPath = path.join(dir, entry.name);
        if (entry.isDirectory()) {
            files = files.concat(getAllSourceFiles(fullPath));
        } else if (entry.isFile() && entry.name.endsWith('.ts')) {
            files.push(fullPath);
        }
    }
    return files;
}

const sourceFiles = getAllSourceFiles(srcDir);

// create a TypeScript program from all .ts files
const tsProgram = ts.createProgram(sourceFiles, {
    allowJs: false,
    declaration: true,
    emitDeclarationOnly: true,
    moduleResolution: ts.ModuleResolutionKind.Bundler,
    strict: true,
    noEmit: true,
});

const readmeTemplatePath = path.join(path.dirname(new URL(import.meta.url).pathname), './README.template.md');
const readmeOutPath = path.join(path.dirname(new URL(import.meta.url).pathname), '../README.md');

let readmeText = fs.readFileSync(readmeTemplatePath, 'utf-8');

/* <RenderType type="import('crashcat').TypeName" /> */
const renderTypeRegex = /<RenderType\s+type=["']import\(['"]crashcat['"]\)\.(\w+)["']\s*\/>/g;
readmeText = readmeText.replace(renderTypeRegex, (fullMatch, typeName) => {
    const typeDef = getType(typeName);
    if (!typeDef) {
        console.warn(`Type ${typeName} not found`);
        return fullMatch;
    }
    return `\`\`\`ts\n${typeDef}\n\`\`\``;
});

/* <RenderSource type="import('crashcat').TypeName" /> */
const renderSourceRegex = /<RenderSource\s+type=["']import\(['"]crashcat['"]\)\.(\w+)["']\s*\/>/g;
readmeText = readmeText.replace(renderSourceRegex, (fullMatch, typeName) => {
    const typeDef = getSource(typeName);
    if (!typeDef) {
        console.warn(`Type ${typeName} not found`);
        return fullMatch;
    }
    return `\`\`\`ts\n${typeDef}\n\`\`\``;
});

/* <Snippet source="./snippets/file.ts" select="group" /> */
const snippetRegex = /<Snippet\s+source=["'](.+?)["']\s+select=["'](.+?)["']\s*\/>/g;
readmeText = readmeText.replace(snippetRegex, (fullMatch, sourcePath, groupName) => {
    const absSourcePath = path.join(path.dirname(new URL(import.meta.url).pathname), sourcePath);
    if (!fs.existsSync(absSourcePath)) {
        console.warn(`Snippet source file not found: ${absSourcePath}`);
        return fullMatch;
    }
    const sourceText = fs.readFileSync(absSourcePath, 'utf-8');

    // extract the selected group and its indentation
    const groupRegex = new RegExp(
        String.raw`^([ \t]*)\/\*[ \t]*SNIPPET_START:[ \t]*${groupName}[ \t]*\*\/[\r\n]+([\s\S]*?)[ \t]*^\1\/\*[ \t]*SNIPPET_END:[ \t]*${groupName}[ \t]*\*\/`,
        'm',
    );
    const match = groupRegex.exec(sourceText);
    if (!match) {
        console.warn(`Snippet group '${groupName}' not found in ${sourcePath}`);
        return fullMatch;
    }
    const baseIndent = match[1] || '';
    let snippetCode = match[2];
    // Remove the detected indentation from all lines
    if (baseIndent) {
        snippetCode = snippetCode.replace(new RegExp(`^${baseIndent}`, 'gm'), '');
    }
    // Remove any leading/trailing blank lines
    snippetCode = snippetCode.replace(/^\s*\n|\n\s*$/g, '');
    return `\`\`\`ts\n${snippetCode}\n\`\`\``;
});

/* write result */
fs.writeFileSync(readmeOutPath, readmeText, 'utf-8');

/* utils */
function getSource(typeName) {
    let found = null;
    function visit(node, fileText) {
        // Types, interfaces, classes
        if (
            (ts.isTypeAliasDeclaration(node) || ts.isInterfaceDeclaration(node) || ts.isClassDeclaration(node)) &&
            node.name &&
            node.name.text === typeName
        ) {
            found = fileText.slice(node.getFullStart(), node.getEnd());
        }
        // Exported function declarations
        if (
            ts.isFunctionDeclaration(node) &&
            node.name &&
            node.name.text === typeName &&
            node.modifiers &&
            node.modifiers.some((m) => m.kind === ts.SyntaxKind.ExportKeyword)
        ) {
            found = fileText.slice(node.getFullStart(), node.getEnd());
        }
        // Exported const expressions
        if (
            ts.isVariableStatement(node) &&
            node.modifiers &&
            node.modifiers.some((m) => m.kind === ts.SyntaxKind.ExportKeyword)
        ) {
            for (const decl of node.declarationList.declarations) {
                if (decl.name && ts.isIdentifier(decl.name) && decl.name.text === typeName) {
                    found = fileText.slice(node.getFullStart(), node.getEnd());
                }
            }
        }
        ts.forEachChild(node, (child) => visit(child, fileText));
    }

    // search all files for the type or function
    for (const file of sourceFiles) {
        const sf = tsProgram.getSourceFile(file);
        if (sf) {
            const fileText = sf.getFullText();
            visit(sf, fileText);
        }
        if (found) break;
    }

    return found ? found.trimStart() : null;
}

function getType(typeName, sourceFile = null) {
    const checker = tsProgram.getTypeChecker();

    let found = null;
    function visit(node, fileText) {
        // Types, interfaces, classes
        if (
            (ts.isTypeAliasDeclaration(node) || ts.isInterfaceDeclaration(node) || ts.isClassDeclaration(node)) &&
            node.name &&
            node.name.text === typeName
        ) {
            const printer = ts.createPrinter({ newLine: ts.NewLineKind.LineFeed });
            found = printer.printNode(ts.EmitHint.Unspecified, node, node.getSourceFile());
        }
        // Exported function declarations
        if (
            ts.isFunctionDeclaration(node) &&
            node.name &&
            node.name.text === typeName &&
            node.modifiers &&
            node.modifiers.some((m) => m.kind === ts.SyntaxKind.ExportKeyword)
        ) {
            // Get signature
            const sig = checker.getSignatureFromDeclaration(node);
            let sigStr = '';
            if (sig) {
                const printer = ts.createPrinter({ newLine: ts.NewLineKind.LineFeed });
                // Print the function signature as a declaration
                const sigNode = ts.factory.createFunctionDeclaration(
                    node.modifiers,
                    node.asteriskToken,
                    node.name,
                    node.typeParameters,
                    node.parameters,
                    node.type,
                    undefined, // no body
                );
                sigStr = printer.printNode(ts.EmitHint.Unspecified, sigNode, node.getSourceFile());
            }
            found = sigStr;
        }
        if (
            ts.isVariableStatement(node) &&
            node.modifiers &&
            node.modifiers.some((m) => m.kind === ts.SyntaxKind.ExportKeyword)
        ) {
            for (const decl of node.declarationList.declarations) {
                // Exported const function expressions (arrow or function)
                if (
                    decl.name &&
                    ts.isIdentifier(decl.name) &&
                    decl.name.text === typeName &&
                    decl.initializer &&
                    (ts.isFunctionExpression(decl.initializer) || ts.isArrowFunction(decl.initializer))
                ) {
                    // Get JSDoc (if any)
                    const jsDoc = ts
                        .getJSDocCommentsAndTags(node)
                        .map((doc) => fileText.slice(doc.pos, doc.end))
                        .join('');
                    // Print only the signature for getType
                    const func = decl.initializer;
                    const printer = ts.createPrinter({
                        newLine: ts.NewLineKind.LineFeed,
                    });
                    const sigNode = ts.factory.createFunctionDeclaration(
                        [ts.factory.createModifier(ts.SyntaxKind.ExportKeyword)],
                        undefined,
                        decl.name,
                        func.typeParameters,
                        func.parameters,
                        func.type,
                        undefined, // no body
                    );
                    const sigStr = printer.printNode(ts.EmitHint.Unspecified, sigNode, node.getSourceFile());
                    found = (jsDoc ? jsDoc + '\n' : '') + sigStr;
                } else if (decl.name && ts.isIdentifier(decl.name) && decl.name.text === typeName) {
                    // Get JSDoc (if any)
                    const jsDoc = ts
                        .getJSDocCommentsAndTags(node)
                        .map((doc) => fileText.slice(doc.pos, doc.end))
                        .join('');
                    // Print variable declaration
                    const printer = ts.createPrinter({ newLine: ts.NewLineKind.LineFeed });
                    const varNode = ts.factory.createVariableStatement(
                        [ts.factory.createModifier(ts.SyntaxKind.ExportKeyword)],
                        ts.factory.createVariableDeclarationList(
                            [ts.factory.createVariableDeclaration(decl.name, undefined, decl.type, decl.initializer)],
                            node.declarationList.flags,
                        ),
                    );
                    const varStr = printer.printNode(ts.EmitHint.Unspecified, varNode, node.getSourceFile());
                    found = (jsDoc ? jsDoc + '\n' : '') + varStr;
                }
            }
        }
        ts.forEachChild(node, (child) => visit(child, fileText));
    }

    // If a specific source file is provided, only search that file
    // Otherwise, search all files for the type or function
    const filesToSearch = sourceFile ? [sourceFile] : sourceFiles.map(file => tsProgram.getSourceFile(file)).filter(Boolean);
    
    for (const sf of filesToSearch) {
        const fileText = sf.getFullText();
        visit(sf, fileText);
        if (found) break;
    }

    return found;
}
