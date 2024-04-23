use std::{collections::HashMap, env::args, process::exit};

fn main() {
    if args().count() == 1 {
        println!(
            r#"usage: fetchdebs <package1>,<package2>,...
                 <packagesFile>@<https://some.prefix/somewhere/>
                 <packagesFile>@..."#
        );
        println!();
        println!("Prints a nix expression to stdout that fetches");
        println!("the given packages and their dependency closure.");
        exit(1);
    }

    let packages = args()
        .nth(1)
        .unwrap()
        .split(',')
        .map(|pkgname| pkgname.to_string())
        .collect::<Vec<_>>();
    let mut all_package_data = HashMap::<String, HashMap<String, String>>::new();

    for package_arg in args().skip(2) {
        let (filename, url_prefix) = package_arg.split_once('@').unwrap();

        let data = std::fs::read(filename).expect("Failed to open for reading");
        let decompressed = if filename.ends_with(".xz") {
            lzma::decompress(&data).expect("Invalid LZMA data")
        } else {
            data
        };
        let string = String::from_utf8(decompressed).expect("Invalid UTF-8");
        let ctrl = debcontrol::parse_str(&string).expect("debctrl error");

        for pkg in ctrl.into_iter() {
            let mut pkg_hashmap = pkg
                .fields
                .into_iter()
                .map(|field| (field.name.to_string(), field.value))
                .collect::<HashMap<String, String>>();
            pkg_hashmap.insert("_prefix".to_string(), url_prefix.to_string());

            let name = pkg_hashmap["Package"].to_string();

            if let Some(provides) = pkg_hashmap.get("Provides") {
                // Insert a pseudo-package for every package that's "provided".
                for prov in provides.split(", ") {
                    let prov = prov.split(|c| c == ' ' || c == ':').nth(0).unwrap();
                    all_package_data.insert(
                        prov.to_string(),
                        [
                            ("Package".to_string(), prov.to_string()),
                            ("Depends".to_string(), name.to_string()),
                        ]
                        .into_iter()
                        .collect(),
                    );
                }
            }

            all_package_data.insert(name, pkg_hashmap);
        }
    }

    let mut to_process = packages;
    let mut required_packages = Vec::<String>::new();
    while !to_process.is_empty() {
        let pkg_name = to_process.pop().unwrap();
        required_packages.push(pkg_name.clone());

        if let Some(data) = all_package_data.get(&pkg_name) {
            if let Some(depends) = data.get("Depends") {
                for dep in depends.split(", ") {
                    // Strip everything past the first space or colon
                    let dep = dep.split(|c| c == ' ' || c == ':').nth(0).unwrap();
                    if required_packages.iter().find(|pkg| *pkg == dep).is_none()
                        && to_process.iter().find(|pkg| *pkg == dep).is_none()
                    {
                        to_process.push(dep.to_string());
                    }
                }
            }
        } else {
            panic!("Missing package {pkg_name}");
        }
    }

    required_packages.sort();
    // println!("{:#?}", required_packages);

    println!("{{ fetchurl }}:");
    println!("[");
    for pkg in required_packages.into_iter() {
        let data = &all_package_data[&pkg];
        if let Some(filename) = data.get("Filename") {
            let prefix = &data["_prefix"];
            let sha256 = &data["SHA256"];

            println!("  (fetchurl {{");
            println!("    url = \"{prefix}{filename}\";");
            println!("    sha256 = \"{sha256}\";");
            println!("  }})");
        }
    }
    println!("]");

    // Use the OS garbage collector :P
    exit(0);
    // It actually does make the program exit faster in debug mode.
}
