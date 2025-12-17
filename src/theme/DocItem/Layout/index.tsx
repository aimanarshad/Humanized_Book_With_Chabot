import React from "react";
// Import the original component we are wrapping
import LayoutOriginal from "@theme-original/DocItem/Layout";
// 👈 STEP 1: Import the necessary hook
import { useDoc } from "@docusaurus/theme-common/internal"; 


export default function DocItemLayout(props) {
  // 👈 STEP 2: Use the hook to safely access document metadata
  const { metadata } = useDoc();

  // 👈 STEP 3: Safely get the ID. If it's undefined, set a safe string.
  const docId = metadata?.id ?? 'unknown-doc-id';
  
  // You can now optionally use the ID for custom body classes or other logic here.
  // The important part is that the variable is defined, preventing the ReferenceError.

  // Pass all original props to the original component
  return (
    <>
      <LayoutOriginal {...props} />
    </>
  );
}