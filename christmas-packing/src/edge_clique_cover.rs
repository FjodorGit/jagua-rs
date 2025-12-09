use std::collections::HashSet;

pub fn edge_clique_cover(n: usize, edges: &[(u32, u32)]) -> Vec<Vec<usize>> {
    // Build adjacency sets
    let mut adj: Vec<HashSet<usize>> = vec![HashSet::new(); n];
    for &(u, v) in edges {
        adj[u as usize].insert(v as usize);
        adj[v as usize].insert(u as usize);
    }

    let mut cover: Vec<Vec<usize>> = Vec::new();

    // Process nodes in order
    for i in 0..n {
        // W = neighbors j < i (edges that need to be covered)
        let mut w: HashSet<usize> = adj[i].iter().filter(|&&j| j < i).copied().collect();

        if w.is_empty() {
            // No edges to cover, create singleton clique
            cover.push(vec![i]);
            continue;
        }

        // Try to add i to existing cliques
        for clique in cover.iter_mut() {
            if can_include(i, clique, &adj) {
                clique.push(i);
                // Remove covered edges from w
                for &j in clique.iter() {
                    w.remove(&j);
                }
                if w.is_empty() {
                    break;
                }
            }
        }

        // For remaining uncovered edges, create new cliques
        while !w.is_empty() {
            // Find existing clique with maximum intersection with w
            let maximal_clique = find_maximal_clique(&cover, &w);

            // Create new clique from intersection + i
            let mut new_clique: Vec<usize> = match maximal_clique {
                Some(clique) => clique.iter().filter(|j| w.contains(j)).copied().collect(),
                None => vec![*w.iter().next().unwrap()],
            };
            new_clique.push(i);

            // Remove covered edges from w
            for &j in &new_clique {
                w.remove(&j);
            }

            cover.push(new_clique);
        }
    }

    cover
}

fn can_include(node: usize, clique: &[usize], adj: &[HashSet<usize>]) -> bool {
    clique.iter().all(|&k| adj[node].contains(&k))
}

fn find_maximal_clique<'a>(cover: &'a [Vec<usize>], w: &HashSet<usize>) -> Option<&'a Vec<usize>> {
    cover
        .iter()
        .max_by_key(|clique| clique.iter().filter(|j| w.contains(j)).count())
}
